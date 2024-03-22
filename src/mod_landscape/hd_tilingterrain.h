/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2009 Jan Reucker (original author)
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

#ifndef HD_TILINGTERRAIN_H
#define HD_TILINGTERRAIN_H

#include "heightdata.h"
#include <plib/ssg.h>

#define SIZE_GRID_PLANES      150
#define SIZE_CELL_GRID_PLANES 20

typedef struct
{
  float x;
  float y;
} myPoint2D;


class HD_TilingTerrain : public HeightData
{
  public:
    HD_TilingTerrain(ssgRoot* SceneGraph);
  
    ~HD_TilingTerrain();
  
    /**
     *  Get the height at a distinct point, in local coordinates, unit is ft
     *
     *  \param x_north  x coordinate (x positive == north)
     *  \param y_east   y coordinate (y positive == east)
     *
     *  \return terrain height at this point in ft
     */
    float getHeight(float x_north, float y_east);

    /**
     *  Get height and plane equation at x|y, in local coordinates, unit is ft
     *
     *  \param x_north  x coordinate (x positive == north)
     *  \param y_east   y coordinate (y positive == east)
     *  \param tplane this is where the plane equation will be stored
     *  \return terrain height at this point in ft
     */
    float getHeightAndPlane(float x_north, float y_east, float tplane[4]);
    
  private:
    void tiling_terrain(ssgEntity * e, sgMat4 xform);

    // test si point x,y, dans la projection du trianle p1, p2,p3 
    // sur plan horizontal
    int on_triangle(float x, float y, float *p1,float *p2, float *p3);

    float Isleft(myPoint2D p1, myPoint2D p2, myPoint2D p);

    ssgVertexArray* tile_table[SIZE_GRID_PLANES][SIZE_GRID_PLANES];
};

#endif // HD_TILINGTERRAIN_H
