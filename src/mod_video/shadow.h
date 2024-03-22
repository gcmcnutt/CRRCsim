/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2011 JOel Lienard (original author)
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
  
/** \file shadow.h
 * 
 *  -choice of shadow algorithm
 *  -class definition of shadow volume algorithm
 */

#ifndef SHADOW_H_
#define SHADOW_H_

//define shadow algorithm
#define SHADOW_NONE       0 // no shadows
#define SHADOW_VOLUME     1 // shadow volume algorithm
#define SHADOW_TYPE       SHADOW_VOLUME

#if (SHADOW_TYPE==SHADOW_VOLUME)

namespace Video
{

void shadowVolumeDrawShadows();


class leafGeometryData
{
  public: 
    leafGeometryData *next;
    int nt;
    short *neighbour;
    bool *lighted, *silhouette;
    sgVec3 *normal;

    leafGeometryData(int nt = 0);
    ~leafGeometryData();
    leafGeometryData* getNext(){ return next;};
    void setNext(leafGeometryData *data){ next = data; };   
};


class ShadowVolume : public ssgVtxTable
{
  public: 
    ShadowVolume(ssgEntity *ent);
    ShadowVolume(ssgEntity *ent, sgMat4 m);
    ~ShadowVolume();
    void update();
    void update(sgMat4 m);
    void reset();
    
  private:
    float cone_length;
    bool updated;
    sgMat4 model_trans;
    ssgEntity *model;
    ssgVertexArray *vertices;
    ssgColourArray *shadow_clr;
    ssgSimpleState *shadow_state;
    leafGeometryData *rootData;    

    void newShadowVolume(ssgEntity *ent, sgMat4 m);
    void modelProperties(leafGeometryData* &data, ssgEntity *ent, sgMat4 xform);
    void modelSilhouette(leafGeometryData* &data, ssgEntity *ent, sgMat4 xform, sgVec3 light);
    void modelShadowVolume(leafGeometryData* &data, ssgEntity *ent, sgMat4 xform, sgVec3 apex);
};

}// end namespace Video::

#endif //(SHADOW_TYPE==SHADOW_VOLUME)
#endif // SHADOW_H_
