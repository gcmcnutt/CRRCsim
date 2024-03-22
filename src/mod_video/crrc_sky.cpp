/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 *   Copyright (C) 2004 - 2009 Jan Reucker (original author)
 *   Copyright (C) 2008 Jens Wilhelm Wulf
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

/** \file crrc_sky.cpp
 *
 *  Definition of a set of sky rendering classes.
 */

#include <stdexcept>
#include <iostream>

#include "crrc_sky.h"
#include "crrc_graphics.h"
#include "../mod_misc/filesystools.h"

// Default ambient light settings
#define DEFAULT_AMBIENT_R     (0.5)
#define DEFAULT_AMBIENT_G     (0.5)
#define DEFAULT_AMBIENT_B     (0.5)

// Default sun settings
#define DEFAULT_SUN_LENSFLARE (1)
#define DEFAULT_SUN_AZIMUTH   (210)
#define DEFAULT_SUN_ELEVATION (25)
#define DEFAULT_SUN_R         (1.0)
#define DEFAULT_SUN_G         (1.0)
#define DEFAULT_SUN_B         (1.0)

// Default fog settings
#define DEFAULT_FOG_DENSITY   (0.2)
#define DEFAULT_FOG_R         (0.85)
#define DEFAULT_FOG_G         (0.90)
#define DEFAULT_FOG_B         (0.95)

namespace Video
{
/** A pointer to the current sky visualization */
static SkyRenderer        *sky = NULL;

/** A copy of the sky parameters (to be able to recreate the visualization
 *  without reloading the scenery file).
 */
static SimpleXMLTransfer  *sky_par = NULL;

/**
 *  A list of XML attributes that contain the
 *  skybox filenames, in the correct order for
 *  the call to the SkyBox ctor.
 */
static const char *boxtex_attribs[] =
{
  "north.filename",
  "south.filename",
  "west.filename",
  "east.filename",
  "up.filename",
  "down.filename"
};

/**
 *  sun flare effect data
 */
static ssgTransform  *sun = NULL;
static ssgaLensFlare *sunFlare = NULL;

/**
 *  Setup the sky from the given XML configuration section.
 *
 *  Creates the sky from parameters read from the scenery file
 *  and additional config file parameters. An existing sky
 *  object wil be deleted.
 *
 *  \param xml      XML configuration section for the sky renderer object
 *                  (pass NULL to reload the existing sky)
 *  \return true for success
 */
SkyRenderer* setup_sky(SimpleXMLTransfer *xml)
{
  // creat a sun flare effect if it does not already exist
  if (sunFlare == NULL)
  {
    sunFlare = new ssgaLensFlare();
    sun = new ssgTransform();
    sun->addKid(sunFlare);
    // make it permanent
    sun->ref();
  }
  
  // create sky from scratch or reload?
  if (xml != NULL)
  {
    // create new sky by copying the configuration
    if (sky_par != NULL)
    {
      delete sky_par;
    }
    sky_par = new SimpleXMLTransfer(xml);
  }
  
  if (sky_par != NULL)
  {
    // delete an existing sky renderer object
    if (sky != NULL)
    {
      delete sky;
      sky = NULL;
    }

    std::string sky_type = sky_par->attribute("type", "original");
    
    try
    {
      if (sky_type == "original")
      {
        // create a CRRCSkyDome
        std::string texture = sky_par->attribute("texture", "");
        float radius = (float)sky_par->getDouble("radius", 8000.0);
        if (texture != "")
        {
          // find full path for the texture
          texture = FileSysTools::getDataPath(texture);
        }
        sky = new CRRCSkyDome(sky_par, texture.c_str(), radius);
      }
      else if (sky_type == "box")
      {
        // create a SkyBox
        char *textures[6] = { NULL, NULL, NULL, NULL, NULL, NULL };
        std::string current_texture;

        float size = (float)sky_par->getDouble("size", 10.0);
        SimpleXMLTransfer *tex = sky_par->getChild("textures", true);

        // get up to six texture names for the box faces
        for (int i = 0; i < 6; i++)
        {
          current_texture = tex->getString(boxtex_attribs[i], "");
          if (current_texture != "")
          {
            textures[i] = new char[current_texture.length() + 1];
            strcpy(textures[i], current_texture.c_str());
          }
        }

        // get the desired texture offset from the main config file
        // (as it is related to the OpenGL implementation rather
        // than the scenery itself)
        float texoffset = cfgfile->getDouble("video.skybox.texture_offset", 0.0);

        sky = new SkyBox(sky_par, (const char**)textures, size, texoffset);

        for (int i = 0; i < 6; i++)
        {
          delete [] textures[i];
        }
      }
      else if (sky_type == "pano")
      {
        // create a CRRCPanoDome
        std::string texture = sky_par->attribute("texture", "");
        float radius = (float)sky_par->getDouble("radius", 10.0);
        if (texture != "")
        {
          // find full path for the texture
          texture = FileSysTools::getDataPath(texture);
        }
        sky = new CRRCPanoDome(sky_par, texture.c_str(), radius);
      }
      else
      {
        // unknown type, create a default CRRCSkyDome
        fprintf(stderr, "Unknown sky type \"%s\" in scenery file, using default.\n", sky_type.c_str());
        sky = new CRRCSkyDome(sky_par, FileSysTools::getDataPath("textures/clouds.rgb").c_str(),
                                 8000.0);
      }
    }
    catch (std::runtime_error& e)
    {
      std::string s = "Unable to create sky visualization:\n";
      s += e.what();
      std::cerr << s << std::endl;
    }
  }
  return sky;
}


/**
 * Draw the sky visualization
 */
void draw_sky(sgVec3 *campos, double dt)
{
  if (sky != NULL)
  {
    sky->update(campos, dt);
    sky->preDraw();
  }
}


/**
 * Deallocate all sky resources
 */
void cleanup_sky()
{
  ssgDeRefDelete(sun);
  delete sky;
  sky = NULL;
  delete sky_par;
  sky_par = NULL;
}


/**
 *  This constructor reads all common skybox parameters
 *  from the xml description file.
**/
SkyRenderer::SkyRenderer(SimpleXMLTransfer *xml)
{
  // read and set ambient light color
  {
    SimpleXMLTransfer *amb = xml->getChild("ambient", true);
    float r, g, b;
        
    r = amb->getDouble("color.r", DEFAULT_AMBIENT_R);   
    g = amb->getDouble("color.g", DEFAULT_AMBIENT_G);   
    b = amb->getDouble("color.b", DEFAULT_AMBIENT_B);   
    sgSetVec4(vSkyAmbientColor, r, g, b, 1.0);    
  }       
      
  // read and set sun position and light color
  {
    SimpleXMLTransfer *sun = xml->getChild("sun", true);
    float azm, ele, x, y, z, r, g, b;
    
    // get sun azimuth and elevation angle in degrees
    // azimuth is 0 north, 90 east.
    // Default are set according to standard skybox images
    azm = sun->getDouble("position.azimuth", DEFAULT_SUN_AZIMUTH);      
    ele = sun->getDouble("position.elevation", DEFAULT_SUN_ELEVATION);   

    // convert azimuth and elevation into (opengl) 
    // scene reference system where:
    //   +Z is S
    //   +X is E
    //   +Y is up
    azm = SG_DEGREES_TO_RADIANS*(azm - 90.);
    ele = SG_DEGREES_TO_RADIANS*ele;
    y = sin(ele);
    x = cos(ele)*cos(azm);
    z = cos(ele)*sin(azm);
    sgSetVec3(vSkySunPosition, x, y, z);
    
    r = sun->getDouble("color.r", DEFAULT_SUN_R);   
    g = sun->getDouble("color.g", DEFAULT_SUN_G);   
    b = sun->getDouble("color.b", DEFAULT_SUN_B);   
    sgSetVec4(vSkySunColor, r, g, b, 1.0);    
  }       
      
  // read and set fog density and color
  {
    SimpleXMLTransfer *haze = xml->getChild("haze", true);
    float dens, r, g, b;
    
    // get fog (actually haze) density and color.
    // Default are set according to standard skybox images
    // fog density from 0 to 1 is mapped into the reasonable
    // range 0 .. 0.001 (using opengl GL_EXP2 fog mode)
    dens = haze->getDouble("density", DEFAULT_FOG_DENSITY);
    dens = dens > 1.0 ? 1.0 : dens < 0.0 ? 0.0 : dens;
    flSkyFogDensity = 0.001*dens;
    
    r = haze->getDouble("color.r", DEFAULT_FOG_R);   
    g = haze->getDouble("color.g", DEFAULT_FOG_G);   
    b = haze->getDouble("color.b", DEFAULT_FOG_B);   
    sgSetVec4(vSkyFogColor, r, g, b, 1.0);
  }
}


/**
 *  DRAW_MODE can be: 0 (draw two triangles per face)
 *                    1 (draw one quad per face)
 */
#define DRAW_MODE 1

/**
 *  Choose a GL primitive depending on the draw mode
 */
#if DRAW_MODE == 0
#define SKYBOX_PRIMITIVES   GL_TRIANGLES
#elif DRAW_MODE == 1
#define SKYBOX_PRIMITIVES   GL_QUADS
#endif

/**
 *  The sky box faces don't form a perfect cube, they
 *  overlap at the edges to reduce floating point glitches.
 *  Credits and thanks go to Danny Chapman for the hint.
 */
#define FACE_OVERLAP  (0.2f)

/**
 * The _vertices are stored as integers because I need to
 * test them for equality later (bad thing to do with
 * floats!). The exact floating point values will be
 * calculated in the ctor.
 */
static const int _vertices[8][3] =
{
  {  1,  1, -1 },
  {  1,  1,  1 },
  {  1, -1,  1 },
  {  1, -1, -1 },
  { -1,  1, -1 },
  { -1,  1,  1 },
  { -1, -1,  1 },
  { -1, -1, -1 }
};

static const int _faces[6][4] =
{
  { 7, 3, 0, 4 },  // north
  { 2, 6, 5, 1 },  // south
  { 6, 7, 4, 5 },  // east
  { 3, 2, 1, 0 },  // west
  { 4, 0, 1, 5 },  // up
  { 6, 2, 3, 7 }   // down
};


/**
 * The _texels are stored as integers because I need to
 * compare them with 0 and 1 later (bad thing to do with
 * floats!). The exact floating point values will be
 * calculated in the ctor.
 */
#if (DRAW_MODE == 0)
static const int _texels[6][2] =
{
  { 0, 0 },
  { 1, 0 },
  { 1, 1 },
  { 1, 1 },
  { 0, 1 },
  { 0, 0 }
};
#elif (DRAW_MODE == 1)
static const int _texels[4][2] =
{
  { 0, 0 },
  { 1, 0 },
  { 1, 1 },
  { 0, 1 },
};
#endif


/**
 *  This constructor creates a skybox from an array
 *  of texture file names for each cube face.
 *
 *  The texture file name array may contain NULL entries.
 *  In this case the associated face will not be drawn.
 *
 *  The texture file names must be in the following order:
 *  north, south, east, west, north looking 90 deg up,
 *  north looking 90 deg down
 *
 *  The texoffset parameter defines how much
 *  the texels should be moved from the texture edge
 *  towards the center of the texture. Try a value of
 *  0.0009 to solve the ATI problem (visible edges of
 *  the sky box faces).
 *  If there are no visible edges (which should be the
 *  case if the OpenGL implementation works as it should),
 *  leave this value at 0. If the value is set too high,
 *  a noticeable seam will appear at the texture edges.
 *
 *  The size doesn't really matter, as the box will be
 *  drawn without Z-buffering as the first element of
 *  the scene. Therefore it will always appear to be
 *  behind everything else regardless of the actual
 *  vertex positions. Choose a size that fits 
 *  somewhere between near and far clipping plane. The
 *  default of 20.0 should fit many applications.
 *
 *  \param textures   Array of six texture file names
 *  \param size       Size of the skybox
 *  \param texoffset  The texels of the box faces will be
 *                    offset from the texture border by this amount.
 */
SkyBox::SkyBox(SimpleXMLTransfer *xml, const char **textures, float size, float texoffset)
  : SkyRenderer(xml), skyroot(NULL), skyboxtrans(NULL), boxsize(size / 2.0f)
{
  skyroot     = new ssgRoot();
  skyboxtrans = new ssgTransform();
  skyroot->addKid(skyboxtrans);

  // get lens flare effect switch, default to true
  int fUseLensFlare = xml->getInt("sun.fUse_lensFlare", DEFAULT_SUN_LENSFLARE);      
  
  // add sun flare effect
  if (fUseLensFlare)
  {
    // NB: for some reason position vector must have "large"
    // components, not normalised ones
    sgVec3 sun_pos;
    sgScaleVec3(sun_pos, vSkySunPosition, 10000);
    sun->setTransform(sun_pos);
    skyroot->addKid(sun);
  }

  // the texture coordinates are the same for each face
  #if (DRAW_MODE == 0)
  ssgTexCoordArray  *texco    = new ssgTexCoordArray(6);
  for (int i = 0; i < 6; i++)
  {
    sgVec2 texel;
    int raw_u = _texels[i][0];
    int raw_v = _texels[i][1];
    float texel_u, texel_v;
    
    texel_u = (raw_u == 0) ? 
                ((float)raw_u + texoffset - 0.5f*FACE_OVERLAP) 
              : ((float)raw_u + 0.5f*FACE_OVERLAP - texoffset);
    texel_v = (raw_v == 0) ? 
                ((float)raw_v + texoffset - 0.5f*FACE_OVERLAP) 
              : ((float)raw_v + 0.5f*FACE_OVERLAP - texoffset);
    
    sgSetVec2(texel, texel_u, texel_v);
    texco->add(texel);
  }
  #elif (DRAW_MODE == 1)
  ssgTexCoordArray  *texco    = new ssgTexCoordArray(4);
  for (int i = 0; i < 4; i++)
  {
    sgVec2 texel;
    int raw_u = _texels[i][0];
    int raw_v = _texels[i][1];
    float texel_u, texel_v;
    
    texel_u = (raw_u == 0) ? 
                ((float)raw_u + texoffset - 0.5f*FACE_OVERLAP) 
              : ((float)raw_u + 0.5f*FACE_OVERLAP - texoffset);
    texel_v = (raw_v == 0) ? 
                ((float)raw_v + texoffset - 0.5f*FACE_OVERLAP) 
              : ((float)raw_v + 0.5f*FACE_OVERLAP - texoffset);
    
    sgSetVec2(texel, texel_u, texel_v);
    texco->add(texel);
  }
  #endif
  
  ssgVertexArray  *vertices;
  ssgLeaf         *vtable;
  ssgSimpleState  *state;
  
  // set up the faces
  for (int i = 0; i < 6; i++)
  {
    if (textures[i] != NULL)
    {
      vertices = new ssgVertexArray(6);
      sgVec3 vertex;
      
      // find the constant coordinate of this face
      typedef enum {CONST_X, CONST_Y, CONST_Z} CONST_FACE_T;
      CONST_FACE_T const_coord;
      //~ for (int k = 0; k < 3; k++)
      {
        // test x
        if ( (_vertices[_faces[i][0]][0] == _vertices[_faces[i][1]][0])
              &&
             (_vertices[_faces[i][0]][0] == _vertices[_faces[i][2]][0]) )
        {
          // x remains constant
          const_coord = CONST_X;
        }
        // test y
        else if ( (_vertices[_faces[i][0]][1] == _vertices[_faces[i][1]][1])
                  &&
                (_vertices[_faces[i][0]][1] == _vertices[_faces[i][2]][1]) )
        {
          // y remains constant
          const_coord = CONST_Y;
        }
        else
        {
          // must be z
          const_coord = CONST_Z;
        }
        
      }
      
      #if (DRAW_MODE == 0)
      // draw triangles
      // draw first triangle of the face
      for (int k = 0; k < 3; k++)
      {
        int num = _faces[i][k];
        int vx = _vertices[num][0];
        int vy = _vertices[num][1];
        int vz = _vertices[num][2];
        vertex[0] = (const_coord == CONST_X) ?
                      (float)vx
                    : (float)vx + (vx * FACE_OVERLAP);
        vertex[1] = (const_coord == CONST_Y) ?
                      (float)vy
                    : (float)vy + (vy * FACE_OVERLAP);
        vertex[2] = (const_coord == CONST_Z) ?
                      (float)vz
                    : (float)vz + (vz * FACE_OVERLAP);
        sgScaleVec3(vertex, boxsize);
        vertices->add(vertex);
      }
  
      // draw second triangle of the face
      for (int k = 2; k < 5; k++)
      {
        int num = _faces[i][k % 4];
        int vx = _vertices[num][0];
        int vy = _vertices[num][1];
        int vz = _vertices[num][2];
        vertex[0] = (const_coord == CONST_X) ?
                      (float)vx
                    : (float)vx + (vx * FACE_OVERLAP);
        vertex[1] = (const_coord == CONST_Y) ?
                      (float)vy
                    : (float)vy + (vy * FACE_OVERLAP);
        vertex[2] = (const_coord == CONST_Z) ?
                      (float)vz
                    : (float)vz + (vz * FACE_OVERLAP);
        sgScaleVec3(vertex, boxsize);
        vertices->add(vertex);
      }
      #elif DRAW_MODE == 1
      // draw quads
      for (int k = 0; k < 4; k++)
      {
        int num = _faces[i][k];
        int vx = _vertices[num][0];
        int vy = _vertices[num][1];
        int vz = _vertices[num][2];
        vertex[0] = (const_coord == CONST_X) ?
                      (float)vx
                    : (float)vx + (vx * FACE_OVERLAP);
        vertex[1] = (const_coord == CONST_Y) ?
                      (float)vy
                    : (float)vy + (vy * FACE_OVERLAP);
        vertex[2] = (const_coord == CONST_Z) ?
                      (float)vz
                    : (float)vz + (vz * FACE_OVERLAP);
        sgScaleVec3(vertex, boxsize);
        vertices->add(vertex);
      }
      #endif

      state = new ssgSimpleState();
      state->setTexture(FileSysTools::getDataPath(textures[i]).c_str(), false, false);
      state->setShadeModel(GL_FLAT);
      state->enable(GL_TEXTURE_2D);
      state->disable(GL_COLOR_MATERIAL);
      state->disable(GL_BLEND);
      state->disable(GL_LIGHTING);
      
      
      vtable = new ssgVtxTable ( SKYBOX_PRIMITIVES,
                                 vertices,
                                 NULL,
                                 texco,
                                 NULL);
      
      vtable->setState(state);
      skyboxtrans->addKid(vtable);
    }
  }
}


/**
 *  Delete all ressources associated with the skybox.
 *
 */
SkyBox::~SkyBox()
{
  skyroot->removeAllKids();
  delete skyroot;
}


/**
 *  To give the perfect illusion, the skybox must always
 *  stay at a fixed position related to the camera.
 *
 *  Call this each time the camera's xyz position changes.
 *
 *  \param campos Current camera position.
 *  \param dt     Elapsed time since last update. Not used.
 */
void SkyBox::update(sgVec3 *campos, double dt)
{
  skyboxtrans->setTransform(*campos);
}


/**
 *  Cull away all box faces outside the frustum and draw
 *  the remaining faces.
 *
 *  This method should be called each frame before the rest
 *  of the scene is drawn. After drawing the sky box, the
 *  depth buffer will be cleared.
 */
void SkyBox::preDraw()
{
  glEnable(GL_DEPTH_TEST);
  ssgCullAndDraw(skyroot);
  glClear(GL_DEPTH_BUFFER_BIT);
}


/* ******************************************************** */
// Default sky dome colors
#define DEFAULT_DOME_R         (0.77)
#define DEFAULT_DOME_G         (0.92)
#define DEFAULT_DOME_B         (0.99)

#define DEFAULT_DOME_FOG_R     (0.9)
#define DEFAULT_DOME_FOG_G     (0.9)
#define DEFAULT_DOME_FOG_B     (0.9)

#define DEFAULT_DOME_CLOUD_R   (0.95)
#define DEFAULT_DOME_CLOUD_G   (0.95)
#define DEFAULT_DOME_CLOUD_B   (0.95)

#if 0

SkyDome::SkyDome(SimpleXMLTransfer *xml, float r, sgVec4 domecol, sgVec4 fogcol, sgVec4 cloudcol)
 : SkyRenderer(xml), theSky(NULL), radius(r)
{
  theSky = new ssgaSky();

  // eliminate warnings
  sgdSetVec3(planetsDummy, radius+1000.0, radius+1000.0, radius+1000.0);
  sgdSetVec3(starsDummy, radius+1000.0, radius+1000.0, radius+1000.0);
  
  theSky->build(radius, radius, 0, &planetsDummy, 0, &starsDummy);
  
  sgCopyVec4(skyDomeCol, domecol);
  sgCopyVec4(skyFogCol, fogcol);
  sgCopyVec4(skyCloudCol, cloudcol);
  
  repaint();
  //~ theSun = theSky->addBody(NULL, "textures/halo.rgba", 5000, 80000, true);
      
  //~ clouds[0] =  theSky -> addCloud (
                          //~ "textures/scattered.rgba", // texture
                          //~ 100000,            // span
                          //~ 2000,             // elevation,
                          //~ 100,              // thickness
                          //~ 100 );            // transition
  //~ clouds[0] -> setSpeed ( 50 ) ;
  //~ clouds[0] -> setDirection ( 45 ) ;
      
}


SkyDome::~SkyDome()
{
  delete theSky;
}


void SkyDome::repaint()
{
  theSky->repaint(skyDomeCol, skyFogCol, skyCloudCol,
                   45*SG_DEGREES_TO_RADIANS,
                   0, &planetsDummy,
                   0, &starsDummy);
}


/**
 *  Update cloud movement.
 *  Call this frequently enough to give smooth cloud movement.
 *
 *  \param campos Current camera position.
 *  \param dt     Elapsed time since last update.
 */
void SkyDome::update(sgVec3 *campos, double dt)
{
  theSky->repositionFlat(*campos, 0, dt );
}

#endif


/* ******************************************************** */

/** 
 *  The constructor creates a sky dome with a given radius
 *  and cloud texture.
 *
 *  The cloud texture must be a greyscale image. It will be
 *  applied as a luminance texture that blends the
 *  dome color with white.
 *
 *  \param cloud_texture Name of the cloud texture. Pass in
 *                       an empty string or a NULL pointer to
 *                       fall back to an untextured sky dome.
 *  \param r Radius of the sky dome.
 */
CRRCSkyDome::CRRCSkyDome(SimpleXMLTransfer *xml, const char *cloud_texture, float r)
  : SkyRenderer(xml), skyroot(NULL), skyboxtrans(NULL), radius(r)
{
  skyroot     = new ssgRoot();
  skyboxtrans = new ssgTransform();
  skyroot->addKid(skyboxtrans);

  // get lens flare effect switch, default to true
  int fUseLensFlare = xml->getInt("sun.fUse_lensFlare", DEFAULT_SUN_LENSFLARE);      
  
  // add sun flare effect
  if (fUseLensFlare)
  {
    // NB: for some reason position vector must have "large"
    // components, not normalised ones
    sgVec3 sun_pos;
    sgScaleVec3(sun_pos, vSkySunPosition, 10000);
    sun->setTransform(sun_pos);
    skyroot->addKid(sun);
  }

  repaint(cloud_texture);
}


/**
 *  The destructor.
 *
 */
CRRCSkyDome::~CRRCSkyDome()
{
  skyroot->removeAllKids();
  delete skyroot;
}


/**
 *  To give the perfect illusion, the skybox must always
 *  stay at a fixed position related to the camera.
 *
 *  Call this each time the camera's xyz position changes.
 *
 *  \param campos Current camera position.
 *  \param dt     Elapsed time since last update. Not used.
 */
void CRRCSkyDome::update(sgVec3 *campos, double dt)
{
  skyboxtrans->setTransform(*campos);
}


/**
 *  The sky dome is drawn without z-buffer writes to make
 *  it the most distant object in the scenery.
 *  Call this method each frame before the rest of the scenery
 *  is drawn. After drawing the sky, the depth buffer will
 *  be cleared.
 */
void CRRCSkyDome::preDraw()
{
  glEnable(GL_DEPTH_TEST);
  ssgCullAndDraw(skyroot);
  glClear(GL_DEPTH_BUFFER_BIT);
}


/**
 *  Build the vertex tables that represent the sky dome.
 *  This method is the "work-horse" of the class. It's a
 *  conversion of the original CRRCsim sky dome to
 *  SSG.
 *
 *  \param cloud_texture Name of the cloud texture. Pass in
 *                       an empty string or a NULL pointer to
 *                       fall back to an untextured sky dome.
 */
void CRRCSkyDome::repaint(const char *texture)
{
  float   INNER_RADIUS;
  float   MIDDLE_RADIUS;
  float   OUTER_RADIUS;
  float   BOTTOM_RADIUS;
  float   CENTER_ELEV;
  float   INNER_ELEV;
  float   MIDDLE_ELEV;
  float   OUTER_ELEV;
  float   BOTTOM_ELEV;
  float   inner_vertex[12][3];  /**< Coordinates of sky sphere points */
  float   middle_vertex[12][3];
  float   outer_vertex[12][3];
  float   bottom_vertex[12][3];
  float   inner_texture[12][2]; /**< Texture coordinates of sky sphere points */
  float   middle_texture[12][2];
  float   outer_texture[12][2];
  float   bottom_texture[12][2];
  
  ssgVtxTable       *table;
  ssgVertexArray    *vertices;
  ssgTexCoordArray  *texels;
  ssgColourArray    *colors;
  ssgSimpleState    *state;
  sgVec3            vertex;
  sgVec2            texel;

  // Colors will be used if no texture was specified.
  // To avoid endless if()'s, we always create the
  // color array but only use it if no texture is
  // specified.
  sgVec4  sky_color;
  sgSetVec4(sky_color, 0.121, 0.355, 0.637, 1.0);
  
  sgVec4  inner_color;
  sgSetVec4(inner_color, 0.207, 0.457, 0.734, 1.0);
  
  sgVec4  middle_color;
  sgSetVec4(middle_color, 0.441, 0.679, 0.933, 1.0);

  sgVec4  outer_color;
  sgSetVec4(outer_color, 0.613, 0.753, 0.933, 1.0);

  INNER_RADIUS  = radius * sin(30 * SG_DEGREES_TO_RADIANS);
  MIDDLE_RADIUS = radius * sin(60 * SG_DEGREES_TO_RADIANS);
  OUTER_RADIUS  = radius;
  BOTTOM_RADIUS = radius;
  CENTER_ELEV   = radius;
  INNER_ELEV    = radius * cos(30 * SG_DEGREES_TO_RADIANS);
  MIDDLE_ELEV   = radius * cos(60 * SG_DEGREES_TO_RADIANS);
  OUTER_ELEV    = 0;
  BOTTOM_ELEV   = -200;

  // transformation matrix from SSG into CRRCsim's coordinate system
  sgMat4 it = {  {1.0,  0.0,  0.0,  0.0},
                 {0.0,  0.0, -1.0,  0.0},
                 {0.0,  1.0,  0.0,  0.0},
                 {0.0,  0.0,  0.0,  1.0}  };

  bool use_textures = true;
  if ((*texture == '\0') || (texture == NULL))
  {
    use_textures = false;
  }

                 
  float diameter = radius * 2;

  for ( int i = 0; i < 12; i++ )
  {
    float theta = (i * 30.0) * SG_DEGREES_TO_RADIANS;
    
    inner_vertex[i][0]  = cos(theta) * INNER_RADIUS;
    inner_vertex[i][2]  = sin(theta) * INNER_RADIUS;
    inner_vertex[i][1]  = INNER_ELEV;
    inner_texture[i][0] = (inner_vertex[i][0] / diameter) + 0.5;
    inner_texture[i][1] = (inner_vertex[i][2] / diameter) + 0.5;

    middle_vertex[i][0] = cos((double)theta) * MIDDLE_RADIUS;
    middle_vertex[i][2] = sin((double)theta) * MIDDLE_RADIUS;
    middle_vertex[i][1] = MIDDLE_ELEV;
    middle_texture[i][0]=(middle_vertex[i][0] / diameter) + 0.5;
    middle_texture[i][1]=(middle_vertex[i][2] / diameter) + 0.5;

    outer_vertex[i][0] = cos((double)theta) * OUTER_RADIUS;
    outer_vertex[i][2] = sin((double)theta) * OUTER_RADIUS;
    outer_vertex[i][1] = OUTER_ELEV;
    outer_texture[i][0]=(outer_vertex[i][0] / diameter) + 0.5;
    outer_texture[i][1]=(outer_vertex[i][2] / diameter) + 0.5;


    bottom_vertex[i][0] = cos((double)theta) * BOTTOM_RADIUS;
    bottom_vertex[i][2] = sin((double)theta) * BOTTOM_RADIUS;
    bottom_vertex[i][1] = BOTTOM_ELEV;
    bottom_texture[i][0]=(bottom_vertex[i][0] / diameter) + 0.5;
    bottom_texture[i][1]=(bottom_vertex[i][2] / diameter) + 0.5;
  }
  
  state = new ssgSimpleState();
  if (use_textures)
  {
    fprintf(stdout, "Creating CRRCSkyDome with texture \"%s\"\n", texture);
    state->setTexture(texture);
    state->enable(GL_TEXTURE_2D);
  }
  else
  {
    // empty string --> no texture
    state->disable(GL_TEXTURE_2D);
    fprintf(stdout, "Creating untextured CRRCSkyDome\n");
  }
  state->setShadeModel(GL_SMOOTH);
  state->enable(GL_COLOR_MATERIAL);
  state->setColourMaterial(GL_AMBIENT_AND_DIFFUSE);
  state->disable(GL_BLEND);
  state->disable(GL_LIGHTING);
  state->disable(GL_CULL_FACE);
  
  // The top cap
  vertices  = new ssgVertexArray();
  texels    = new ssgTexCoordArray();
  colors    = new ssgColourArray();

  sgSetVec3(vertex, 0.0f, 0.0f, CENTER_ELEV);
  vertices->add(vertex);
  sgSetVec2(texel, 0.5f, 0.5f);
  texels->add(texel);
  colors->add(sky_color);
  for (int i = 11; i >= 0; i--)
  {
    sgSetVec3(vertex, inner_vertex[i][0], inner_vertex[i][2], inner_vertex[i][1]);
    vertices->add(vertex);
    sgSetVec2(texel, inner_texture[i][0], inner_texture[i][1]);
    texels->add(texel);
    colors->add(inner_color);
  }
  sgSetVec3(vertex,  inner_vertex[11][0], inner_vertex[11][2], inner_vertex[11][1]);
  vertices->add(vertex);
  sgSetVec2(texel, inner_texture[11][0], inner_texture[11][1]);
  texels->add(texel);
  colors->add(inner_color);
  
  if (use_textures)
  {
    delete colors;
    colors = NULL;
  }
  else
  {
    delete texels;
    texels = NULL;
  }
  
  table = new ssgVtxTable ( GL_TRIANGLE_FAN,
                             vertices,
                             NULL,
                             texels,
                             colors);

  table->setState(state);
  table->transform(it);
  table->recalcBSphere();
  skyboxtrans->addKid(table);
  
  // first ring
  vertices  = new ssgVertexArray();
  texels    = new ssgTexCoordArray();
  colors    = new ssgColourArray();

  for (int i = 0; i < 12; i++)
  {
    sgSetVec3(vertex,  middle_vertex[i][0], middle_vertex[i][2], middle_vertex[i][1]);
    vertices->add(vertex);
    sgSetVec2(texel, middle_texture[i][0], middle_texture[i][1]);
    texels->add(texel);
    colors->add(middle_color);
    
    sgSetVec3(vertex,  inner_vertex[i][0], inner_vertex[i][2], inner_vertex[i][1]);
    vertices->add(vertex);
    sgSetVec2(texel, inner_texture[i][0], inner_texture[i][1]);
    texels->add(texel);
    colors->add(inner_color);
  }
  sgSetVec3(vertex,  middle_vertex[0][0], middle_vertex[0][2], middle_vertex[0][1]);
  vertices->add(vertex);
  sgSetVec2(texel, middle_texture[0][0], middle_texture[0][1]);
  texels->add(texel);
  colors->add(middle_color);
  sgSetVec3(vertex,  inner_vertex[0][0], inner_vertex[0][2], inner_vertex[0][1]);
  vertices->add(vertex);
  sgSetVec2(texel, inner_texture[0][0], inner_texture[0][1]);
  texels->add(texel);
  colors->add(inner_color);
  
  if (use_textures)
  {
    delete colors;
    colors = NULL;
  }
  else
  {
    delete texels;
    texels = NULL;
  }
  
  table = new ssgVtxTable(GL_TRIANGLE_STRIP,
                          vertices,
                          NULL,
                          texels,
                          colors);

  table->setState(state);
  table->transform(it);
  table->recalcBSphere();
  skyboxtrans->addKid(table);
  
  // second ring
  vertices  = new ssgVertexArray();
  texels    = new ssgTexCoordArray();
  colors    = new ssgColourArray();

  for (int i = 0; i < 12; i++)
  {
    sgSetVec3(vertex, outer_vertex[i][0], outer_vertex[i][2], outer_vertex[i][1]);
    vertices->add(vertex);
    sgSetVec2(texel, outer_texture[i][0], outer_texture[i][1]);
    texels->add(texel);
    colors->add(outer_color);
    
    sgSetVec3(vertex, middle_vertex[i][0], middle_vertex[i][2], middle_vertex[i][1]);
    vertices->add(vertex);
    sgSetVec2(texel, middle_texture[i][0], middle_texture[i][1]);
    texels->add(texel);
    colors->add(middle_color);
  }
  sgSetVec3(vertex, outer_vertex[0][0], outer_vertex[0][2], outer_vertex[0][1]);
  vertices->add(vertex);
  sgSetVec2(texel, outer_texture[0][0], outer_texture[0][1]);
  texels->add(texel);
  colors->add(outer_color);
  sgSetVec3(vertex, middle_vertex[0][0], middle_vertex[0][2], middle_vertex[0][1]);
  vertices->add(vertex);
  sgSetVec2(texel, middle_texture[0][0], middle_texture[0][1]);
  texels->add(texel);
  colors->add(middle_color);

  if (use_textures)
  {
    delete colors;
    colors = NULL;
  }
  else
  {
    delete texels;
    texels = NULL;
  }
  
  table = new ssgVtxTable(GL_TRIANGLE_STRIP,
                          vertices,
                          NULL,
                          texels,
                          colors);

  table->setState(state);
  table->transform(it);
  table->recalcBSphere();
  skyboxtrans->addKid(table);

  // bottom ring
  vertices  = new ssgVertexArray();
  texels    = new ssgTexCoordArray();
  colors    = new ssgColourArray();

  for (int i = 0; i < 12; i++)
  {
    sgSetVec3(vertex, bottom_vertex[i][0], bottom_vertex[i][2], bottom_vertex[i][1]);
    vertices->add(vertex);
    sgSetVec2(texel, bottom_texture[i][0], bottom_texture[i][1]);
    texels->add(texel);
    colors->add(outer_color);
    sgSetVec3(vertex, outer_vertex[i][0], outer_vertex[i][2], outer_vertex[i][1]);
    vertices->add(vertex);
    sgSetVec2(texel, outer_texture[i][0], outer_texture[i][1]);
    texels->add(texel);
    colors->add(outer_color);
  }
  sgSetVec3(vertex, bottom_vertex[0][0], bottom_vertex[0][2], bottom_vertex[0][1]);
  vertices->add(vertex);
  sgSetVec2(texel, bottom_texture[0][0], bottom_texture[0][1]);
  texels->add(texel);
  colors->add(outer_color);
  sgSetVec3(vertex, outer_vertex[0][0], outer_vertex[0][2], outer_vertex[0][1]);
  vertices->add(vertex);
  sgSetVec2(texel, outer_texture[0][0], outer_texture[0][1]);
  texels->add(texel);
  colors->add(outer_color);
  
  if (use_textures)
  {
    delete colors;
    colors = NULL;
  }
  else
  {
    delete texels;
    texels = NULL;
  }
  
  table = new ssgVtxTable(GL_TRIANGLE_STRIP,
                          vertices,
                          NULL,
                          texels,
                          colors);

  table->setState(state);
  table->transform(it);
  table->recalcBSphere();
  skyboxtrans->addKid(table);
}


/* ******************************************************** */

/** 
 *  The constructor creates a panorama dome with a given radius
 *  and texture.
 *
 *  \param texture  Name of the panorama texture.
 *  \param r        Radius of the sky dome.
 */
CRRCPanoDome::CRRCPanoDome(SimpleXMLTransfer *xml, const char *texture, float r)
 : SkyRenderer(xml), texfilename(texture), radius(r)
{
  if (texfilename == "")
  {
    std::string msg = "CRRCPanoDome: no texture name";
    throw std::runtime_error(msg);
  }

  init_textures();
  init_vertices();
}


/**
 *  The destructor.
 */
CRRCPanoDome::~CRRCPanoDome()
{
  //~ delete panotex;
  //~ delete state;
}


/**
 *  The panorama dome is drawn without z-buffer writes to make
 *  it the most distant object in the scenery.
 *  Call this method each frame before the rest of the scenery
 *  is drawn. After drawing the dome, the depth buffer will
 *  be cleared.
 */
void CRRCPanoDome::preDraw()
{
  glEnable(GL_DEPTH_TEST);
  //ssgCullAndDraw(skyroot);
  render();
  glClear(GL_DEPTH_BUFFER_BIT);
}



void CRRCPanoDome::init_textures()
{
  panotex = new ssgTexture(texfilename.c_str(), false, false);
  if (panotex == NULL)
  {
    std::string msg = "CRRCPanoDome: Unable to load panorama texture ";
    msg += texfilename;
    throw std::runtime_error(msg);
  }
  
  // setup rendering state
  state = new ssgSimpleState () ;
  state -> setTexture ( panotex ) ;
  state -> disable  ( GL_LIGHTING ) ;
  state -> enable   ( GL_TEXTURE_2D ) ;
  state -> disable  ( GL_COLOR_MATERIAL ) ;
  state -> disable  ( GL_CULL_FACE ) ;
  state -> disable  ( GL_BLEND ) ;
  
  float tex_base = (2048.0f-768.0f) / 2048.0f;
  float ring_height = 768.0f / 2048.0f / PANO_NUM_RINGS;
  float slice_width = 1.0f / PANO_NUM_SLICES;
  
  for (int i = 0; i < PANO_NUM_RINGS; i++)
  {
    // constant "v" coordinate on this ring
    float ring_v = tex_base + i * ring_height;
    for (int k = 0; k < PANO_NUM_SLICES; k++)
    {
      texco[i][k][0] = k * slice_width;
      texco[i][k][1] = ring_v;
      //~ printf("i: %d  k:%d  u: %f   v: %f\n", i, k, texco[i][k][0], texco[i][k][1]);
    }
  }
}


void CRRCPanoDome::init_vertices()
{
  for (int i = 0; i < PANO_NUM_RINGS; i++)
  {
    // constant data for this ring
    float ring_radius = radius * cos(((float)i * 135.0f) * SG_DEGREES_TO_RADIANS / (float)(PANO_NUM_RINGS) - 45.0f * SG_DEGREES_TO_RADIANS);
    float elev = radius * sin(((float)i * 135.0f) * SG_DEGREES_TO_RADIANS / (float)PANO_NUM_RINGS - 45.0f * SG_DEGREES_TO_RADIANS);

    printf("i: %d   elev: %f   radius: %f\n", i, elev, ring_radius);

    for (int k = 0; k < PANO_NUM_SLICES; k++)
    {
      float angle = k * 360.0f * SG_DEGREES_TO_RADIANS / (float)PANO_NUM_SLICES;
      #ifdef SSG_COORDINATES
      vertices[i][k][SG_Z] = elev;
      vertices[i][k][SG_X] = ring_radius * cos(angle);
      vertices[i][k][SG_Y] = ring_radius * sin(angle);
      #else
      vertices[i][k][SG_Y] = elev + PANO_CAM_HEIGHT;
      vertices[i][k][SG_X] = ring_radius * cos(angle);
      vertices[i][k][SG_Z] = -ring_radius * sin(angle);
      #endif

      //~ printf("Vertex %f %f %f\n", vertices[i][k][SG_X], vertices[i][k][SG_Y], vertices[i][k][SG_Z]);
    }
  }
}


void CRRCPanoDome::render()
{
  ssgContext *context = Video::getGlobalRenderingContext();

  glMatrixMode ( GL_PROJECTION ) ;
  context->loadProjectionMatrix () ;

  glMatrixMode ( GL_MODELVIEW ) ;
  glLoadIdentity () ;
  context->loadModelviewMatrix();

  state -> apply();

  #if (PANO_RENDER_MODE == 0)
  glBegin(GL_POINTS);
  glPointSize(3);
  glColor3f(1.0, 1.0, 1.0);
  for (int i = 0; i < (PANO_NUM_RINGS); i++)
  {
    for (int k = 0; k < PANO_NUM_SLICES; k++)
    {
      glTexCoord2f(texco[i][k][0], texco[i][k][1]);
      glVertex3f(vertices[i][k][SG_X], vertices[i][k][SG_Y], vertices[i][k][SG_Z]);
    }
  }
  glEnd();
  glPointSize(1);
  
  #elif (PANO_RENDER_MODE == 1)
  glBegin(GL_QUADS);
  glColor3f(1.0, 1.0, 1.0);
  for (int i = 0; i < (PANO_NUM_RINGS - 1); i++)
  {
    for (int k = 0; k < (PANO_NUM_SLICES - 1); k++)
    {
      glTexCoord2f(texco[i][k][0], texco[i][k][1]);
      glVertex3f(vertices[i][k][SG_X], vertices[i][k][SG_Y], vertices[i][k][SG_Z]);

      glTexCoord2f(texco[i][k+1][0], texco[i][k+1][1]);
      glVertex3f(vertices[i][k+1][SG_X], vertices[i][k+1][SG_Y], vertices[i][k+1][SG_Z]);

      glTexCoord2f(texco[i+1][k+1][0], texco[i+1][k+1][1]);
      glVertex3f(vertices[i+1][k+1][SG_X], vertices[i+1][k+1][SG_Y], vertices[i+1][k+1][SG_Z]);

      glTexCoord2f(texco[i+1][k][0], texco[i+1][k][1]);
      glVertex3f(vertices[i+1][k][SG_X], vertices[i+1][k][SG_Y], vertices[i+1][k][SG_Z]);
    }
    
    // last quad in a ring
    glTexCoord2f(texco[i][PANO_NUM_SLICES - 1][0], texco[i][PANO_NUM_SLICES - 1][1]);
    glVertex3f(vertices[i][PANO_NUM_SLICES - 1][SG_X], vertices[i][PANO_NUM_SLICES - 1][SG_Y], vertices[i][PANO_NUM_SLICES - 1][SG_Z]);

    glTexCoord2f(1.0, texco[i][0][1]);
    glVertex3f(vertices[i][0][SG_X], vertices[i][0][SG_Y], vertices[i][0][SG_Z]);

    glTexCoord2f(1.0, texco[i+1][0][1]);
    glVertex3f(vertices[i+1][0][SG_X], vertices[i+1][0][SG_Y], vertices[i+1][0][SG_Z]);

    glTexCoord2f(texco[i+1][PANO_NUM_SLICES - 1][0], texco[i+1][PANO_NUM_SLICES - 1][1]);
    glVertex3f(vertices[i+1][PANO_NUM_SLICES - 1][SG_X], vertices[i+1][PANO_NUM_SLICES - 1][SG_Y], vertices[i+1][PANO_NUM_SLICES - 1][SG_Z]);
  }
  glEnd();
  // triangle fan as top cap
  glBegin(GL_TRIANGLE_FAN);
  glTexCoord2f(0.5, 1.0);
  #ifdef SSG_COORDINATES
  glVertex3f(0.0, 0.0, radius);
  #else
  glVertex3f(0.0, radius + PANO_CAM_HEIGHT, 0.0);
  #endif
  for (int k = 0; k < PANO_NUM_SLICES; k++)
  {
    glTexCoord2f(texco[PANO_NUM_RINGS - 1][k][0], texco[PANO_NUM_RINGS - 1][k][1]);
    glVertex3f(vertices[PANO_NUM_RINGS - 1][k][SG_X], vertices[PANO_NUM_RINGS - 1][k][SG_Y], vertices[PANO_NUM_RINGS - 1][k][SG_Z]);
  }
  //~ glTexCoord2f(texco[PANO_NUM_RINGS - 1][0][0], texco[PANO_NUM_RINGS - 1][0][1]);
  glTexCoord2f(1.0, texco[PANO_NUM_RINGS - 1][0][1]);
  glVertex3f(vertices[PANO_NUM_RINGS - 1][0][SG_X], vertices[PANO_NUM_RINGS - 1][0][SG_Y], vertices[PANO_NUM_RINGS - 1][0][SG_Z]);
  glEnd();
  #endif
}

} // namespace Video
