/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 *   Copyright (C) 2009 Jan Reucker (original author)
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

#ifndef HD_TABULATEDTERRAIN_H
#define HD_TABULATEDTERRAIN_H

#include "heightdata.h"
#include <plib/ssg.h>

#define SIZE_GRID_PLANES        150
#define SIZE_CELL_GRID_PLANES   20


class HD_TabulatedTerrain : public HeightData
{
  public:
    HD_TabulatedTerrain(ssgRoot* SceneGraph);
  
    ~HD_TabulatedTerrain();
  
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
    void make_tab_HeightAndPlane(); 
    float tab_Plane[SIZE_GRID_PLANES+1][SIZE_GRID_PLANES+1][4];
    float tab_HOT[SIZE_GRID_PLANES+1][SIZE_GRID_PLANES+1];
};

#endif // HD_TABULATEDTERRAIN_H
