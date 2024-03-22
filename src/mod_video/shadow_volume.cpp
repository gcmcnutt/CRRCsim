/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2011 Joel Lienard (original author)
 * Copyright (C) 2016 Luca Gasparini
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
  
/** \file shadow_volume.cpp
 * 
 *  Implementation of shadow algorithm.
 *
 */
 
/* references: 
-Improving Shadows and Reflections via the Stencil Buffer
Mark J. Kilgard (NVIDIA Corporation) : http://www.opengl.org/resources/code/samples/mjktips/rts/index.html
-Practical and Robust Stenciled Shadow Volumes for Hardware-Accelerated Rendering
Authors: Cass Everitt, Mark J. Kilgard : http://arxiv.org/abs/cs/0301002
-Shadow Volumes, http://www.paulsprojects.net, paul@paulsprojects.net

Objectif:
 -The algorithm used initially by CRRCSIM works correctly only if the ground is plan. Otherwise, certain parts of the shadow are not drawn because they are under the ground, others are in the air.

Choices:
shadow volumes/shadow-map:
 -shadow volume to avoid the drawbacks of shado-map (quantification, not supported opengl-extension)

*/


#include <list>
#include "crrc_ssgutils.h"
#include "crrc_graphics.h"
#include "crrc_sky.h"
#include "../global.h"
#include "shadow.h"
#include "../mod_landscape/crrc_scenery.h"

#if (SHADOW_TYPE==SHADOW_VOLUME)

#define SHADOW_VOLUME_VISIBLE   0     // 1 to see the shadowVolume (TEST)
#define EPS_VTX                 1e-6  // vertex merge distance tolerance

namespace Video
{

static float shadowVolumeDarkness = 0.0; // computed value of shadow darkness
  
  
/**
 *  To draw the shadow, after the stencil buffer has been prepared,
 *  it is only required to draw a gray rectangle over the full screen
 *  with stencil test on.
 *
 */
void shadowVolumeDrawShadows()
{
  // as soon as the sky is created define shadow darkness 
  // approximately based on sun light intensity
  if (shadowVolumeDarkness == 0.0)
  {
    sgVec4 amb_light, sun_light;
    Global::scenery->getAmbientColor(amb_light);
    Global::scenery->getSunColor(sun_light);
    float max_sun, max_amb;
    max_sun = max_amb = 0.;
    for (int i = 0; i < 3; i++)
    {
      max_amb = amb_light[i] > max_amb ? amb_light[i] : max_amb;
      max_sun = sun_light[i] > max_sun ? sun_light[i] : max_sun;
    }
    shadowVolumeDarkness = (max_sun - max_amb);
  }

  // draw the shadows marked in the stencil buffer 
  glStencilMask(0xFF);
  glEnable(GL_STENCIL_TEST);
  glStencilFunc(GL_NOTEQUAL, 0x0, 0xFF);
  glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
  
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_TEXTURE_2D);
  glDisable(GL_LIGHTING);

  glMatrixMode(GL_PROJECTION);
  glPushMatrix(); 
  glLoadIdentity();
  gluOrtho2D(0.0,1.0,0.0,1.0);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  glBegin(GL_QUADS);
  glColor4f(0.0,0.0,0.0,shadowVolumeDarkness);
  glVertex2f(0.0,0.0);
  glVertex2f(1.0,0.0);
  glVertex2f(1.0,1.0);
  glVertex2f(0.0,1.0);
  glEnd();      

  glMatrixMode(GL_PROJECTION);
  glPopMatrix(); 
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();

  glStencilMask(0xFF);
  glDisable(GL_STENCIL_TEST);
  glDisable(GL_BLEND);
  glEnable(GL_DEPTH_TEST);
}
    
/**
 *  Mark the stencil buffer where an odd (usually 1) number of shadow
 *  volume faces overlap the scene. These area will be "in the shadow".
 *  This function does all the GL state manipulations and the drawing
 *  operations concerning the shadow volume object.
 *  Thus it returns FALSE-
 */
static int shadowVolumePredrawCallback(ssgEntity *ent)
{  
  ssgVtxTable *v = (ssgVtxTable *)ent;

  // prepare GL settings
#if (!SHADOW_VOLUME_VISIBLE)
  glColorMask(0,0,0,0);
#endif
  glEnable(GL_CULL_FACE);
  glEnable(GL_STENCIL_TEST);
  glDepthMask(0); // don't write to the depth buffer
  glStencilFunc(GL_ALWAYS, 0x0, 0x0);
  glPolygonOffset(1.0, 2.0);
  
  // first pass, cull back faces,
  // increment stencil buffer on depth pass.
  // Draw volume object using the prepared display list
  glFrontFace(GL_CCW);
  glStencilOp(GL_KEEP, GL_KEEP, GL_INCR);
  glCallList(v->getDListIndex());

  // second pass, cull front faces,
  // decrement stencil buffer on depth pass.
  // Draw volume object using the prepared display list
  glFrontFace(GL_CW);
  glStencilOp(GL_KEEP, GL_KEEP, GL_DECR);
  glCallList(v->getDListIndex());
    
  // restore base settings
  glPolygonOffset(0.0, 0.0);
  glDisable(GL_CULL_FACE);
  glDisable(GL_STENCIL_TEST);
  glColorMask(1,1,1,1);
  glDepthMask(1);
  glFrontFace(GL_CCW);

  // nothing else to be done by ssg..
  return false;
}


/******************************************************************/


leafGeometryData::leafGeometryData(int n)
{
  next = NULL;
  nt = n;
  if (nt)
  {
    // Create space for face normals
    normal = new sgVec3[nt];
    // Create space for connectivity data
    neighbour = new short[nt*3];
    // Create space for the "is facing light" booleans
    lighted = new bool[nt];
    // Create space for "is silhouette edge" booleans
    silhouette = new bool[nt*3];    
  }
}


leafGeometryData::~leafGeometryData()
{
  if (nt)
  {
    delete normal;
    delete neighbour;
    delete lighted;
    delete silhouette;
  }
}


/******************************************************************/


ShadowVolume::ShadowVolume(ssgEntity *ent)
: ssgVtxTable(GL_TRIANGLES, NULL, NULL, NULL, NULL)
{
  sgMat4 m;
  sgMakeIdentMat4(m);
  newShadowVolume(ent, m);
}


ShadowVolume::ShadowVolume(ssgEntity *ent, sgMat4 m)
: ssgVtxTable(GL_TRIANGLES, NULL, NULL, NULL, NULL)
{
  newShadowVolume(ent, m);
}


void ShadowVolume::newShadowVolume(ssgEntity *ent, sgMat4 m)
{
  model = ent;
  sgCopyMat4(model_trans, m);
  updated = false;
  
  leafGeometryData* data;
  rootData = new leafGeometryData();
  
  // estimate the (approximate) lenght of the shadow cone cast
  // by the model from its maximum size (bounding sphere radius)
  // and sun's apparent radius of 0.25deg
  sgSphere *bsphere = model->getBSphere();
  float radius = bsphere->getRadius();
  cone_length = radius/tan(0.25*SG_DEGREES_TO_RADIANS);  

  // pre-compute model's constant properties
  sgMat4 xform;
  sgMakeIdentMat4(xform);
  modelProperties(data = rootData, model, xform);

  // shadow volume objects will never take part in collision/height
  clrTraversalMaskBits(SSGTRAV_HOT | SSGTRAV_LOS);

  // drawing of shadow volume will be managed entirely by this
	setCallback(SSG_CALLBACK_PREDRAW, shadowVolumePredrawCallback);
  
  // empty vertex table, to be filled by update()
  vertices = new ssgVertexArray();
	setVertices(vertices);
  
  // shadow volume walls color
#if (SHADOW_VOLUME_VISIBLE)
  shadow_clr = new ssgColourArray();
  sgVec4 color = {1.0, 0.0, 0.0, 0.5};
  shadow_clr->add(color);
  setColours(shadow_clr);
#endif
}


ShadowVolume::~ShadowVolume()
{
  leafGeometryData* leafData = rootData->getNext();
  
  while (leafData)
  {
    leafGeometryData* nextData = leafData->getNext();
    delete leafData;
    leafData = nextData;
  }  
  delete rootData;  
}


void ShadowVolume::modelProperties(leafGeometryData* &data, ssgEntity *ent, sgMat4 xform)
{
  if (ent->isAKindOf(ssgTypeBranch()))
  {
    ssgBranch *branch = (ssgBranch *)ent;
    for (int i = 0; i < branch->getNumKids(); i++)
    {
      modelProperties(data, branch->getKid(i), xform);
    }
  }
  else if (ent->isAKindOf(ssgTypeLeaf()))
  {
    ssgLeaf *leaf = (ssgLeaf *)ent;
    SSGUtil::NodeAttributes attr = SSGUtil::getNodeAttributes(leaf);
    if ((attr.checkAttribute("shadow") != -1)
        && (attr.checkAttribute("visible") != -1))
    {
      int nt = leaf->getNumTriangles();
      
      // allocate data for leaf geometry
      data->next = new leafGeometryData(nt);
      data = data->next;
      
      // allocate temporary face's bounding box data
      sgBox *bbox = new sgBox[nt];
      
      // calculate normal and bounding box for each face
      for (int i = 0; i < data->nt; i++)
      {
        short iv1, iv2, iv3;
        sgVec3 v1, v2, v3;
        leaf->getTriangle(i, &iv1, &iv2, &iv3);
        sgCopyVec3(v1,leaf->getVertex(iv1));
        sgCopyVec3(v2,leaf->getVertex(iv2));
        sgCopyVec3(v3,leaf->getVertex(iv3));
        sgMakeNormal(data->normal[i], v1, v2, v3);
        // add vertices to face's bounding box
        bbox[i].extend(v1);
        bbox[i].extend(v2);
        bbox[i].extend(v3);
      }
      
      // calculate connectivity info for each face
      for (int i = 0; i < data->nt*3; i++)
        data->neighbour[i] = -1;  

      float eps2 = EPS_VTX*EPS_VTX;
      for (int i = 0; i < data->nt-1; i++)
      {
        short iv[3];
        leaf->getTriangle(i, &iv[0], &iv[1], &iv[2]);
        for (int ei = 0; ei < 3; ei++)
        {
          if (data->neighbour[i*3+ei] != -1) continue;
          
          short ei1 = iv[ei];
          short ei2 = iv[(ei+1)%3];
          sgVec3 vi1, vi2;
          sgCopyVec3(vi1,leaf->getVertex(ei1));
          sgCopyVec3(vi2,leaf->getVertex(ei2));
          
          for (int j = i+1; j < data->nt; j++)
          {
            // to speed-up the search for neighbour face check bbox first
            // and only complete the test if the two bbox intersect
            if (bbox[i].intersects(&bbox[j]))
            {
              short jv[3];
              leaf->getTriangle(j, &jv[0], &jv[1], &jv[2]);
              for (int ej = 0; ej < 3; ej++)
              {
                short ej1 = jv[ej];
                short ej2 = jv[(ej+1)%3];
                sgVec3 vj1, vj2;
                sgCopyVec3(vj1,leaf->getVertex(ej1));
                sgCopyVec3(vj2,leaf->getVertex(ej2));
                
                if (
                  ((sgDistanceSquaredVec3(vi1, vj1) < eps2) && (sgDistanceSquaredVec3(vi2, vj2) < eps2)) 
                  ||
                  ((sgDistanceSquaredVec3(vi1, vj2) < eps2) && (sgDistanceSquaredVec3(vi2, vj1) < eps2))
                )
                {
                  data->neighbour[i*3+ei] = j;
                  data->neighbour[j*3+ej] = i;
                }
              }
            }
          }
        }
      }
    }
  }
}


void ShadowVolume::modelSilhouette(leafGeometryData* &data, ssgEntity *ent, sgMat4 xform, sgVec3 light)
{
  if (ent->isAKindOf(ssgTypeBranch()))
  {
    ssgBranch *branch = (ssgBranch *)ent;
    if (ent->isA(ssgTypeTransform()))
    {
      sgMat4 xform1;
      ((ssgTransform *)ent)->getTransform(xform1);
      sgPreMultMat4(xform, xform1);
    }
    sgMat4 local_xform;
    sgCopyMat4(local_xform, xform); // save transformation matrix
    for (int i = 0; i < branch->getNumKids(); i++)
    {
      modelSilhouette(data, branch->getKid(i), xform, light);
      // restore transformation matrix
      sgCopyMat4(xform, local_xform);
    }
  }
  else if (ent->isAKindOf(ssgTypeLeaf()))
  {
    ssgLeaf *leaf = (ssgLeaf *)ent;
    SSGUtil::NodeAttributes attr = SSGUtil::getNodeAttributes(leaf);
    if ((attr.checkAttribute("shadow") != -1)
        && (attr.checkAttribute("visible") != -1))
    {
      data = data->next;

      bool ident = (sgClassifyMat4(xform) == SG_IDENTITY);
      
      // mark faces facing the light
      for (int i = 0; i < data->nt; i++)
      {
        sgVec3 xnormal;
        // if necessary, apply the transform to face normal
        if (ident)
          sgCopyVec3(xnormal, data->normal[i]);
        else
          sgXformVec3(xnormal, data->normal[i], xform);

        // check normal orientation
        if (sgScalarProductVec3(xnormal, light) > 0.0)
          data->lighted[i] = TRUE;
        else
          data->lighted[i] = FALSE;
      }

      // mark silhouette edges
      for (int i = 0; i < data->nt*3; i++)
      {
        if (!data->lighted[i/3])
        {
          data->silhouette[i] = FALSE;
          continue;
        }
        if (data->neighbour[i] == -1 || 
          !data->lighted[data->neighbour[i]])
        {
          data->silhouette[i] = TRUE;
          continue;
        }
        data->silhouette[i] = FALSE;
      }
    }
  }
}


void ShadowVolume::modelShadowVolume(leafGeometryData* &data, ssgEntity *ent, sgMat4 xform, sgVec3 apex)
{
  if (ent->isAKindOf(ssgTypeBranch()))
  {
    ssgBranch *branch = (ssgBranch *)ent;
    if (ent->isA(ssgTypeTransform()))
    {
      sgMat4 xform1;
      ((ssgTransform *)ent)->getTransform(xform1);
      sgPreMultMat4(xform, xform1);
    }
    sgMat4 local_xform;
    sgCopyMat4(local_xform, xform); // save transformation matrix
    for (int i = 0; i < branch->getNumKids(); i++)
    {
      modelShadowVolume(data, branch->getKid(i), xform, apex);
      // restore transformation matrix
      sgCopyMat4(xform, local_xform);
    }
  }
  else if (ent->isAKindOf(ssgTypeLeaf()))
  {
    ssgLeaf *leaf = (ssgLeaf *)ent;
    SSGUtil::NodeAttributes attr = SSGUtil::getNodeAttributes(leaf);
    if ((attr.checkAttribute("shadow") != -1)
        && (attr.checkAttribute("visible") != -1))
    {
      data = data->next;

      bool ident = (sgClassifyMat4(xform) == SG_IDENTITY);
      
      // collect vertices of silhouette edges
      for (int i = 0; i < data->nt; i++)
      {
        if (!data->lighted[i])
          continue;

        short iv[3];
        leaf->getTriangle(i, &iv[0], &iv[1], &iv[2]);

        for (int j = 0; j < 3; j++)
          if (data->silhouette[i*3+j])
          {
            sgVec3 v1, v2;
            // if necessary apply the transform to silhouette vertices
            if (ident)
            {
              sgCopyVec3(v1, leaf->getVertex(iv[(j+1)%3]));
              sgCopyVec3(v2, leaf->getVertex(iv[j]      ));
            }
            else
            {
              sgXformPnt3(v1, leaf->getVertex(iv[(j+1)%3]), xform);
              sgXformPnt3(v2, leaf->getVertex(iv[j]      ), xform);
            }
            
            // add vertices and also add apex vertex
            vertices->add(v1);
            vertices->add(v2);
            vertices->add(apex);
          }
      }
    }
  }
}


void ShadowVolume::update()
{
  if (!updated)
  {
    updated = true;
    
    leafGeometryData* data;

    // inverse of model transformation to transform the
    // light source position in model reference frame 
    sgMat4 m;
    sgTransposeNegateMat4(m, model_trans);

    // apply inverse of transformation matrix from SSG
    // into CRRCsim's coordinate system
    sgMat4 it = { {1.0,  0.0,  0.0,  0.0}, 
                  {0.0,  0.0,  1.0,  0.0},
                  {0.0, -1.0,  0.0,  0.0},
                  {0.0,  0.0,  0.0,  1.0} };
    sgPostMultMat4(m, it);

    // compute light direction in model reference frame
    sgVec3 sun_pos;
    Global::scenery->getSunPosition(sun_pos);
    sgNormaliseVec3(sun_pos);
    sgXformVec3(sun_pos, m);

    // compute silhouette edges viewed from the light source
    sgMat4 xform;
    sgMakeIdentMat4(xform);
    modelSilhouette(data = rootData, model, xform, sun_pos);

    // build shadow volume in model reference frame
    sgVec3 apex;
    sgScaleVec3(apex, sun_pos, -cone_length);
    vertices->rawSetNum(0); // re-use vertices vertexArray
    modelShadowVolume(data = rootData, model, xform, apex);
    recalcBSphere();
    makeDList();
  }
}


void ShadowVolume::update(sgMat4 m)
{
  updated = false;
  sgCopyMat4(model_trans, m);
  
  update();
}


void ShadowVolume::reset()
{
  shadowVolumeDarkness = 0.0;
}

} // end of namespace Video::
#endif //(SHADOW_TYPE==SHADOW_VOLUME)
