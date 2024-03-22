/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 *   Copyright (C) 2000, 2001 Jan Kansky (original author)
 *   Copyright (C) 2004-2010 Jan Reucker
 *   Copyright (C) 2004, 2005, 2006, 2008 Jens Wilhelm Wulf
 *   Copyright (C) 2005 Chris Bayley
 *   Copyright (C) 2005 Lionel Cailler
 *   Copyright (C) 2006 Todd Templeton
 *   Copyright (C) 2009 Joel Lienard
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** \file model_based_scenery.cpp
 *  This file defines a "scenery" class which contains all data
 *  and methods to construct and draw the landscape.
 */

#include <crrc_config.h>

#include <iostream>
#include <iomanip>

#include "../crrc_main.h"
#include "../mod_misc/SimpleXMLTransfer.h"
#include "../mod_misc/filesystools.h"
#include "../mod_misc/ls_constants.h"
#include "model_based_scenery.h"
#include "hd_ssgLOSterrain.h"
#include "hd_tabulatedterrain.h"
#include "hd_tilingterrain.h"
#include "wind_from_terrain.h"
#include "../mod_video/shadow.h"

#if WINDDATA3D != 1
#include "../GUI/crrc_msgbox.h"
#endif

// This module uses some internal SSG stuff from the video module!
#include "../mod_video/crrc_ssgutils.h"

static int translucentTerrainPredrawCallback(ssgEntity *ent)
{
  // translucent terrain objects use alpha blend.
  // This is set for backwards compatibility, but shouldn't
  // be used since even fully transparent areas will mask
  // objects placed behind.
  // It is likely that the object should actually be 
  // specified either as non-terrain or as non-translucent.

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  return TRUE;
}

static int translucentTerrainPostdrawCallback(ssgEntity *ent)
{
  glDisable(GL_BLEND);
  return TRUE;
}

static int transparentTerrainPredrawCallback(ssgEntity *ent)
{
  // a terrain objects may be defined non-traslucent
  // (hence "transparent") to force its rendering as a fully
  // transparent object which anyway "covers" object behind it.
  // It is a usefull option to speed-up rendering of terrain
  // in photo-realistic sceneries.
  // Terrain must be drawn after the skybox is rendered.

  glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
  return TRUE;
}

static int transparentTerrainPostdrawCallback(ssgEntity *ent)
{
  glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
  return TRUE;
}

static int translucentObjectPredrawCallback(ssgEntity *ent)
{
  // translucent, non-terrain, objects, use alpha blend and
  // alpha test to discard almost transparent areas.
  // This allows order-independent transparency rendering of the
  // discarded area, but order backt-to-front is required for 
  // proper translucent blending and to avoid artifacts.

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_ALPHA_TEST);
  glAlphaFunc(GL_GREATER, 0.1f); // discard almost transparent fragments
  return TRUE;
}

static int translucentObjectPostdrawCallback(ssgEntity *ent)
{
  glDisable(GL_BLEND);
  glDisable(GL_ALPHA_TEST);
  return TRUE;
}

static int opaqueObjectPredrawCallback(ssgEntity *ent)
{
  // partially transparent, non-terrain, objects use no blending
  // but alpha test, so that only fully-opaque areas are drawn.
  // This allows order-independent rendering of the resulting opaque
  // objects, althoug front-to-back ordering is faster.
  // Usefull for billboard objects (e.g. trees), since it is
  // faster then using really translucent objects and if fully
  // order independent.

  glEnable(GL_ALPHA_TEST);
  glAlphaFunc(GL_GREATER, 0.99f); // discard non-opaque fragments
  return TRUE;
}

static int opaqueObjectPostdrawCallback(ssgEntity *ent)
{
  glDisable(GL_ALPHA_TEST);
  return TRUE;
}

static int opaqueObject2PassPostdrawCallback(ssgEntity *ent)
{
  opaqueObjectPostdrawCallback(ent);
  
  // pass2..
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDepthMask(GL_FALSE);

  // ..using the display list created for the leaf
  ssgLeaf *leaf = (ssgLeaf *)ent;
  glCallList(leaf->getDListIndex());
  
  glDisable(GL_BLEND);
  glDepthMask(GL_TRUE);
  return TRUE;
}

/****************************************************************************/
/* Model based scenery                                                      */
/****************************************************************************/
ModelBasedScenery::ModelBasedScenery(SimpleXMLTransfer *xml, int sky_variant)
    : Scenery(xml, sky_variant), location(Scenery::MODEL_BASED)
{
  ssgEntity *model = NULL;
  SimpleXMLTransfer *scene = xml->getChild("scene", true);
  getHeight_mode = scene->attributeAsInt("getHeight_mode", DEFAULT_HEIGHT_MODE);
  //std::cout << "----getHeight_mode : " <<  getHeight_mode <<std::endl;

  SceneGraph = new ssgRoot();

  // find all "objects" defined in the file
  int num_children = scene->getChildCount();

  for (int cur_child = 0; cur_child < num_children; cur_child++)
  {
    SimpleXMLTransfer *kid = scene->getChildAt(cur_child);
    // only use "object" tags
    if (kid->getName() == "object")
    {
      std::string filename = kid->attribute("filename", "not_specified");
      bool is_cutout = (kid->attributeAsInt("cutout", 0) != 0);
      bool is_terrain = (kid->attributeAsInt("terrain", 1) != 0);
      bool is_visible = (kid->attributeAsInt("visible", 1) != 0);
      bool allow_translucent = (kid->attributeAsInt("translucent", 1) != 0);
      bool cast_shadow = (kid->attributeAsInt("shadow", 0) != 0);

      // only non-terrain objects can be defined as cutout
      is_cutout = !is_terrain && is_cutout;
      
      // PLIB automatically loads the texture file,
      // but it does not know which directory to use.
      // Where is the object file?
      std::string    of  = FileSysTools::getDataPath("objects/" + filename, TRUE);
      // compile and set relative texture path
      std::string    tp  = of.substr(0, of.length()-filename.length()-1-7) + "textures";
      ssgTexturePath(tp.c_str());

      // load model
      std::cout << "Loading 3D "
                << (is_cutout ? "cutout " : " ")      
                << "object \"" << of.c_str() << "\""
                << " (default:"
                << (is_terrain ? " " : " not part of ") << "terrain,"
                << (is_visible ? " " : " not ") << "visible,"
                << (allow_translucent ? " " : " don't ") << "allow translucent,"
                << (cast_shadow ? " " : " don't ") << "cast shadow)"
                << std::endl;
      model = ssgLoad(of.c_str());
      
      if (model != NULL)
      {
        // The model may contain internal node attributes (e.g. for
        // integrated collision boxes) which override default.
        // Parse these attributes now.
        evaluateNodeAttributes(model, is_terrain, is_visible, allow_translucent);       
        
        // now parse the instances and place the model in the SceneGraph
        for (int cur_instance = 0; cur_instance < kid->getChildCount(); cur_instance++)
        {
          SimpleXMLTransfer *instance = kid->getChildAt(cur_instance);
          if (instance->getName() == "instance")
          {
            addInstance(instance, is_cutout, cast_shadow, model, SceneGraph);
          }
          else if (instance->getName() == "population")
          {
            addPopulation(instance, is_cutout, cast_shadow, model, SceneGraph);
          }
        }
      }
    }
  }
  
  // create actual terrain height model
  if (getHeight_mode == 1)
  {
    heightdata = new HD_TabulatedTerrain(SceneGraph);
  }
  else if (getHeight_mode == 2)
  {
    heightdata = new HD_TilingTerrain(SceneGraph);
  }
  else
  {
    heightdata = new HD_SsgLOSTerrain(SceneGraph);
  }

  //wind
  SimpleXMLTransfer *wind = xml->getChild("wind", true);
  std::string wind_filename = wind->attribute("filename","");
#if WINDDATA3D == 1
  wind_data = 0;//default : no wind_data
  std::string wind_position_unit = wind->attribute("unit","");
  try {
    flDefaultWindDirection = wind->attributeAsInt("direction");
    ImposeWindDirection = true;
  }
  catch (XMLException)
  {
    // if not attribut "direction", normal mode
  }
  
  if (wind_position_unit.compare("m")==0)
  {
    wind_position_coef = FT_TO_M;
  }
  else
  {
    wind_position_coef = 1;
  }
  std::cout << "wind file name :  " << wind_filename.c_str()<< std::endl;
  if (wind_filename.length() > 0)
  {
    wind_filename = FileSysTools::getDataPath(wind_filename);  
    std::cout << "init wind ---------";
    int n = init_wind_data((wind_filename.c_str()));
    std::cout << n << "  points processed" << std::endl;
  }
#else
  if (wind_filename.length() > 0)
  {
    new CGUIMsgBox("Insufficient configuration to read windfields.");
  }
#endif
}

/**
 * Recursively walk a scene graph and evaluate any node
 * attributes placed inside the name strings of leaves.
 *
 */
void ModelBasedScenery::evaluateNodeAttributes(ssgEntity* ent, 
    bool def_is_terrain, bool def_is_visible, bool def_allow_translucent)
{
  if (ent->isAKindOf(ssgTypeLeaf()))
  {
    ssgLeaf *leaf = (ssgLeaf*)ent;

    SSGUtil::NodeAttributes attr = SSGUtil::getNodeAttributes(ent);
    std::string node_name = SSGUtil::getPureNodeName(ent);

    bool is_visible = def_is_visible;
    int attr_visible = attr.checkAttribute("visible");
    if (attr_visible)
    {
      is_visible = attr_visible > 0 ? 1 : 0;
    }
    if (is_visible != def_is_visible)
    {
      std::cout << "  leaf \"" << node_name << "\"" << " is" 
                << (is_visible ? " " : " not ") << "visible"
                << std::endl;
    }
    
    // Objects are made invisible by enclosing in an invisible branch
    // NB: clearing the CULL traversal flag does not work!
    if (!is_visible)
    {
      // This doesn't work. Should it ?
      //ent->clrTraversalMaskBits(SSGTRAV_CULL);
      SSGUtil::spliceBranch(new ssgInvisible(), ent);
    }
    else
    {
      bool is_terrain = def_is_terrain;
      int attr_terrain = attr.checkAttribute("terrain");
      if (attr_terrain)
      {
        is_terrain = attr_terrain > 0 ? 1 : 0;
      }
      if (is_terrain != def_is_terrain)
      {
        std::cout << "  leaf \"" << node_name << "\"" << " is"
                  << (is_terrain ? " " : " not part of ") << "terrain" 
                  << std::endl;
      }
      
      // In PLIB::SSG, intersection testing is done by a tree-walking
      // function. This can be influenced by the tree traversal mask
      // bits. The HOT and LOS flags are cleared for objects that are
      // not part of the terrain, so that the height-of-terrain and
      // line-of-sight algorithms ignore this branch of the tree.
      if (!is_terrain)
      {
        ent->clrTraversalMaskBits(SSGTRAV_HOT | SSGTRAV_LOS);
      }
      
      // check if object is translucent, i.e. has alpha info
      bool is_translucent = leaf->isTranslucent();

      // If object is translucent setup appropriate drawing states
      if (is_translucent)
      {
        std::cout << "  leaf \"" << node_name << "\"" << " is translucent, ";

        bool allow_translucent = def_allow_translucent;
        int attr_translucent = attr.checkAttribute("translucent");
        if (attr_translucent)
        {
          allow_translucent = attr_translucent > 0 ? 1 : 0;
        }

        if (is_terrain)
        {
          // translucent terrain object rendering options:
          // - translucent: alpha blending, no alpha test
          // - fully transparent: no color write, only depth buffer write
          if (allow_translucent)
          {
            leaf->setCallback (SSG_CALLBACK_PREDRAW, translucentTerrainPredrawCallback);
            leaf->setCallback (SSG_CALLBACK_POSTDRAW, translucentTerrainPostdrawCallback);
            std::cout << "rendered translucent";
          }
          else
          {
            leaf->setCallback (SSG_CALLBACK_PREDRAW, transparentTerrainPredrawCallback);
            leaf->setCallback (SSG_CALLBACK_POSTDRAW, transparentTerrainPostdrawCallback);
            std::cout << "rendered fully transparent";
          }
        }
        else
        {
          // translucent non-terrain object rendering options:
          // - translucent: alpha blending, alpha test discards almost transparent areas
          // - partially transparent: no blending, alpha test discards non-opaque areas
          if (allow_translucent)
          {
            leaf->setCallback (SSG_CALLBACK_PREDRAW, translucentObjectPredrawCallback);
            leaf->setCallback (SSG_CALLBACK_POSTDRAW, translucentObjectPostdrawCallback);
            std::cout << "rendered translucent";
          }
          else
          {
            // fast, low-quality, 1-pass rendering
            /*
            leaf->setCallback (SSG_CALLBACK_PREDRAW, opaqueObjectPredrawCallback);
            leaf->setCallback (SSG_CALLBACK_POSTDRAW, opaqueObjectPostdrawCallback);
            */
            // slow, high-quality, 2-pass rendering, using leaf's display list
            if (!leaf->getDListIndex())
              leaf->makeDList();
            leaf->setCallback (SSG_CALLBACK_PREDRAW, opaqueObjectPredrawCallback);
            leaf->setCallback (SSG_CALLBACK_POSTDRAW, opaqueObject2PassPostdrawCallback);
            std::cout << "rendered partially transparent";
          }
        }
        
        std::cout << std::endl;
      }
    }
  }
  else if (ent->isAKindOf(ssgTypeBranch()))
  {
    ssgBranch *branch = (ssgBranch*)ent;

    // continue down the hierarchy
    int kids = branch->getNumKids();
    for (int i = 0; i < kids; i++)
    {
      ssgEntity* currKid = branch->getKid(i);
      evaluateNodeAttributes(currKid, def_is_terrain, def_is_visible, def_allow_translucent);
    }
  }
}

void ModelBasedScenery::addInstance(SimpleXMLTransfer *xml,
     bool is_cutout, bool cast_shadow, ssgEntity *model, ssgBranch *parent)
{        
  sgCoord coord;
  sgMat4 xform;
  
  // try north/east/height first, then fallback to x/y/z
  try
  {
    coord.xyz[SG_X] = xml->attributeAsDouble("east");
  }
  catch (XMLException &e)
  {
    coord.xyz[SG_X] = xml->attributeAsDouble("y", 0.0);
  }
  try
  {
    coord.xyz[SG_Y] = xml->attributeAsDouble("north");
  }
  catch (XMLException &e)
  {
    coord.xyz[SG_Y] = xml->attributeAsDouble("x", 0.0);
  }
  try
  {
    coord.xyz[SG_Z] = xml->attributeAsDouble("height");
  }
  catch (XMLException &e)
  {
    coord.xyz[SG_Z] = xml->attributeAsDouble("z", 0.0);
  }
  coord.hpr[0] = 180 - xml->attributeAsDouble("h", 0.0);
  coord.hpr[1] = -xml->attributeAsDouble("p", 0.0);
  coord.hpr[2] = -xml->attributeAsDouble("r", 0.0);

  // is object height specified above ground level ? 
  bool is_agl = (xml->attributeAsInt("above_ground", 0) != 0);
  if (is_agl)
  {
    HD_SsgLOSTerrain *temp_heightdata = new HD_SsgLOSTerrain(SceneGraph);
    float terrain_height = temp_heightdata->getHeight(coord.xyz[SG_Y], coord.xyz[SG_X]);
    coord.xyz[SG_Z] += terrain_height;
  }
      
  std::cout << std::setprecision(1);
  std::cout << "  Placing instance at " << coord.xyz[SG_X] << ";" << coord.xyz[SG_Y] << ";" << coord.xyz[SG_Z];
  std::cout << std::setprecision(3);
  std::cout << ", orientation " << (180-coord.hpr[0]) << ";" << -coord.hpr[1] << ";" << -coord.hpr[2] << std::endl;  
  std::cout << std::setprecision(6);
  
  sgMakeCoordMat4(xform, &coord);
  addObject(is_cutout, cast_shadow, model, parent, xform);
}

void ModelBasedScenery::addPopulation(SimpleXMLTransfer *xml,
     bool is_cutout, bool cast_shadow, ssgEntity *model, ssgBranch *parent)
{
  int n1, n2;
  float noiseFacD1, noiseFacD2, noiseFacS, noiseFacSX, noiseFacSZ;
  sgVec3 x, s, x0, d1, d2, dir1, dir2;
  sgMat4 xform;
  sgPerlinNoise_2D noiseD1, noiseD2, noiseS, noiseSX, noiseSZ;
  HD_SsgLOSTerrain *temp_heightdata;
  
  // try north/east/height first, then fallback to x/y/z
  try
  {
    x0[SG_X] = xml->attributeAsDouble("east");
    d1[SG_X] = xml->attributeAsDouble("d1_east");
    d2[SG_X] = xml->attributeAsDouble("d2_east");
  }
  catch (XMLException &e)
  {
    x0[SG_X] = xml->attributeAsDouble("y", 0.0);
    d1[SG_X] = xml->attributeAsDouble("d1_y", 0.0);
    d2[SG_X] = xml->attributeAsDouble("d2_y", 0.0);
  }
  try
  {
    x0[SG_Y] = xml->attributeAsDouble("north");
    d1[SG_Y] = xml->attributeAsDouble("d1_north");
    d2[SG_Y] = xml->attributeAsDouble("d2_north");
  }
  catch (XMLException &e)
  {
    x0[SG_Y] = xml->attributeAsDouble("x", 0.0);
    d1[SG_Y] = xml->attributeAsDouble("d1_x", 0.0);
    d2[SG_Y] = xml->attributeAsDouble("d2_x", 0.0);
  }
  try
  {
    x0[SG_Z] = xml->attributeAsDouble("height");
    d1[SG_Z] = xml->attributeAsDouble("d1_height");
    d2[SG_Z] = xml->attributeAsDouble("d2_height");
  }
  catch (XMLException &e)
  {
    x0[SG_Z] = xml->attributeAsDouble("z", 0.0);
    d1[SG_Z] = xml->attributeAsDouble("d1_z", 0.0);
    d2[SG_Z] = xml->attributeAsDouble("d2_z", 0.0);
  }
  
  // distribution noise and object size noise factors
  // NB: double amplitude since Perlin noise return -0.5..+0.5
	noiseFacD1 = 2.0*xml->attributeAsDouble("d1_noise", 0.0);
	noiseFacD2 = 2.0*xml->attributeAsDouble("d2_noise", 0.0);
  noiseFacS = 2.0*xml->attributeAsDouble("s_noise", 0.0);
  noiseFacSX = 2.0*xml->attributeAsDouble("sx_noise", 0.0);
  noiseFacSZ = 2.0*xml->attributeAsDouble("sz_noise", 0.0);
  
  // is object height specified above ground level ? 
  bool is_agl = (xml->attributeAsInt("above_ground", 0) != 1);
  if (is_agl)
    temp_heightdata = new HD_SsgLOSTerrain(SceneGraph);

  if (sgLengthVec3(d1) > 0.0)
    sgNormaliseVec3(dir1, d1);
  else
    sgZeroVec3(dir1);
  if (sgLengthVec3(d2) > 0.0)
    sgNormaliseVec3(dir2, d2);
  else
    sgZeroVec3(dir2);
  
  n1 = xml->attributeAsInt("n1", 1);
  n2 = xml->attributeAsInt("n2", 1);
  float k1 = n1 > 1 ? 1.0/(n1 - 1) : 0.0;
  float k2 = n2 > 1 ? 1.0/(n2 - 1) : 0.0;  
  
  // create a population of objects
  // move along d1 direction first, then one step in d2 and so on
  for (int j = 1; j <= n2; j++)
    for (int i = 1; i <= n1; i++)
    {
      float u = k1*(i-1);
      float v = k2*(j-1);
      
      // compute object "regular" location
      sgCopyVec3(x, x0);
      sgAddScaledVec3(x, d1, u);
      sgAddScaledVec3(x, d2, v);

      // map to 0..255,0..255 to evalute Perlin noise functions
      // NB: Perlin output is within -0.5..+0.5
      sgVec2 uv;
      sgSetVec2(uv, 255.0*u, 255.0*v);
      
      // add location noise
      sgAddScaledVec3(x, dir1, noiseFacD1*noiseD1.getNoise(uv));
      sgAddScaledVec3(x, dir2, noiseFacD2*noiseD2.getNoise(uv));
      
      // optionally set object z as height above ground level
      if (is_agl)
        x[SG_Z] += temp_heightdata->getHeight(x[SG_Y], x[SG_X]);
      
      // set object scale noise (overall, X and Z direction)
      float overallS = 1.0 - noiseFacS*(noiseS.getNoise(uv) - 0.5);
      sgSetVec3(s, 
         overallS*(1.0 + noiseFacSX*noiseSX.getNoise(uv)),
         overallS,
         overallS*(1.0 + noiseFacSZ*noiseSZ.getNoise(uv))
      );

      // define object instance transformation
      // 1 - scale
      sgSetVec4(xform[SG_X], s[SG_X], 0.0, 0.0, 0.0);
      sgSetVec4(xform[SG_Y], 0.0, s[SG_Y], 0.0, 0.0);
      sgSetVec4(xform[SG_Z], 0.0, 0.0, s[SG_Z], 0.0);
      // 2 - translation
      sgSetVec4(xform[SG_W], x[SG_X], x[SG_Y], x[SG_Z], 1.0);
	  
      addObject(is_cutout, cast_shadow, model, parent, xform);
    }
}

void ModelBasedScenery::addObject(bool is_cutout, bool cast_shadow, 
     ssgEntity *model, ssgBranch *parent, sgMat4 xform)
{        
  // transformation matrix from SSG into CRRCsim's coordinate system
  sgMat4 it = {  {1.0,  0.0,  0.0,  0.0},
                 {0.0,  0.0, -1.0,  0.0},
                 {0.0,  1.0,  0.0,  0.0},
                 {0.0,  0.0,  0.0,  1.0}  };
  sgMat4 itInv;
  sgTransposeNegateMat4(itInv, it);

  // object transformation has been originally computed according to
  // the transformation sequence:
  // initial_trans->addKid(model_trans)
  // Now, to make the tree structure of scenery objects equal to that of
  // airplane objects the order of the transformations is inverrted:
  // model_trans->addKid(initial_trans)
  // Thus, we must transform the transformation matrix according to
  // M' = Inv(A) * M * A
  sgPreMultMat4(xform, itInv);
  sgPostMultMat4(xform, it);
  
  ssgTransform *model_trans = new ssgTransform();
  parent->addKid(model_trans);
  model_trans->setTransform(xform);
  ssgTransform *initial_trans = new ssgTransform();
  model_trans->addKid(initial_trans);
  initial_trans->setTransform(it);

  // put the real object in a selectable branch
  // active by default
  ssgSelector *object_branch = new ssgSelector(1);
  object_branch->setName("object_branch");
  object_branch->select(1);
  initial_trans->addKid(object_branch);
  
  // If the model is a cutout then put it into a cutout branch.
  if (is_cutout)
  {
    ssgCutout *cutout_branch = new ssgCutout(FALSE);
    object_branch->addKid(cutout_branch);
    cutout_branch->addKid(model);
  }
  else
    object_branch->addKid(model);
    
  if (cast_shadow)
  {
#if (SHADOW_TYPE==SHADOW_VOLUME)
    // put the shadow volume object in a selectable branch
    // active by default
    ssgSelector *shadow_branch = new ssgSelector(1);
    shadow_branch->setName("shadow_branch");
    shadow_branch->select(1);
    initial_trans->addKid(shadow_branch);
    Video::ShadowVolume *shadow = new Video::ShadowVolume(model, xform);
    shadow_branch->addKid(shadow);
#endif
  }
}

ModelBasedScenery::~ModelBasedScenery()
{
  delete SceneGraph;
  delete heightdata;
#if WINDDATA3D == 1
  if (wind_data)
    delete wind_data;
#endif
}

void ModelBasedScenery::activateSceneTree(ssgEntity *ent, bool fObject)
{
  if (ent->isAKindOf(ssgTypeBranch()))
  {
    if (ent->isA(ssgTypeSelector()))
    {
      // check if selector is an object/shadow selector
      // and activate/deactivate it as required
      if (ent->getName() != NULL)
      {
        ssgSelector *selector = (ssgSelector*)ent;
        std::string name = ent->getName();
        if (name == "object_branch")
          selector->select(fObject);
        else if (name == "shadow_branch")
          selector->select(!fObject);
      }
    }
    
    ssgBranch *branch = (ssgBranch*)ent;

    // continue down the hierarchy
    int kids = branch->getNumKids();
    for (int i = 0; i < kids; i++)
    {
      ssgEntity* currKid = branch->getKid(i);
      activateSceneTree(currKid, fObject);
    }
  }
}

#if (SHADOW_TYPE==SHADOW_VOLUME)
void ModelBasedScenery::updateSceneShadows(ssgEntity *ent)
{
  if (ent->isAKindOf(ssgTypeBranch()))
  {
    ssgBranch *branch = (ssgBranch*)ent;

    if (branch->isA(ssgTypeSelector()))
    {
      // check if selector is a shadow selector
      // and update object shadow volume definition
      if (branch->getName() != NULL)
      {
        std::string name = branch->getName();
        if (name == "shadow_branch")
        {
          Video::ShadowVolume *shadow = (Video::ShadowVolume*)branch->getKid(0);
          shadow->update();
        }
      }
    }

    // continue down the hierarchy
    int kids = branch->getNumKids();
    for (int i = 0; i < kids; i++)
    {
      ssgEntity* currKid = branch->getKid(i);
      updateSceneShadows(currKid);
    }
  }
}
#endif

void ModelBasedScenery::draw(double current_time)
{
  // traverse the scenery tree activating real objects
  // and deactivating shadows
  activateSceneTree(SceneGraph, true);

  // draw the scene
  ssgCullAndDraw(SceneGraph);
}

void ModelBasedScenery::draw_shadows(double current_time)
{
#if (SHADOW_TYPE==SHADOW_VOLUME)
  // traverse the scenery tree deactivating real objects
  // and activating shadows
  activateSceneTree(SceneGraph, false);

  // update objects shadows.
  // On one hand they might, in the future, change with time
  // in case of moving objects.
  // On the other hand sun position was not yet available
  // at the time that scenery objects are first added to the scene
  updateSceneShadows(SceneGraph);
  
  // draw the scene
  ssgCullAndDraw(SceneGraph);
#endif
}

float ModelBasedScenery::getHeight(float x_north, float y_east)
{
  return heightdata->getHeight(x_north, y_east);
}

float ModelBasedScenery::getHeightAndPlane(float x, float y, float tplane[4])
{
  return heightdata->getHeightAndPlane(x, y, tplane);
}

int ModelBasedScenery::getWindComponents(double X, double Y,double Z,
    float *x_wind_velocity, float *y_wind_velocity, float *z_wind_velocity)
{
  // freestream wind velocity
  float flWindVel = cfg->wind->getVelocity();
  float flWindDir = cfg->wind->getDirection()*DEG_TO_RAD;
  *x_wind_velocity = -1 * flWindVel * cos(flWindDir);
  *y_wind_velocity = -1 * flWindVel * sin(flWindDir);
  *z_wind_velocity = 0.;

#if WINDDATA3D == 1
  //import wind data from file
  float x,y,z,vx,vy,vz;
  if (wind_data)
  {
    x =  X * wind_position_coef;
    y =  Y * wind_position_coef;
    z = -Z * wind_position_coef;
    if (find_wind_data(x,y,z,&vx,&vy,&vz))
    {
      // point found
      *x_wind_velocity = vx * flWindVel;
      *y_wind_velocity = vy * flWindVel;
      *z_wind_velocity = vz * flWindVel;
      //std::cout << "----at"<< x<<"  "<<y<<"  "<< z<<"wind components***" << *x_wind_velocity <<"  "<< *y_wind_velocity <<"  "<<  *z_wind_velocity<<std::endl;
      return 0;
    }
    else
    {
      // point not found
      return 1;
   }
  }
  else
#endif
  {
    //default mode
    return wind_from_terrain(X, Y, Z, x_wind_velocity, y_wind_velocity, z_wind_velocity);
  }
}
