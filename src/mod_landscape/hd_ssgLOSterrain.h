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

#ifndef HD_SSGLOSTERRAIN_H
#define HD_SSGLOSTERRAIN_H

#include "heightdata.h"


class HD_SsgLOSTerrain : public HeightData
{
  public:
    HD_SsgLOSTerrain(ssgRoot* SceneGraph);
  
    ~HD_SsgLOSTerrain();
  
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
};

#endif // HD_SSGLOSTERRAIN_H
