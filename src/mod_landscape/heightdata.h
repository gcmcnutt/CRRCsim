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

#ifndef HEIGHTDATA_H
#define HEIGHTDATA_H

#include <plib/ssg.h>
#include "crrc_scenery.h"


/**
 * An abstract base class for height representation classes
 *
 * There are many ways to store and access height data for the landscape.
 * This abstract base class defines the common interface for all specific
 * implementations.
 *
 * Coordinate system for this class and all subclasses:
 * x = positive north, negative south
 * y = positive east, negative west
 * z = positive down, negative up
 *
 * The z-axis might seem to be reverted in this system (because mountain tops
 * have a lower z value than valleys), but sticking to the classical conventions
 * (north and east are positive) the z-axis must point to the center of the earth
 * if the coordinate system is right-handed (thumb points north, index finger east,
 * middle finger down). This is what CRRCsim's FDM refers to as "local coordinates"
 * (or "world coordinates").
 *
 */
class HeightData
{
  public:
    HeightData(ssgRoot* SceneGraph);
    
    /** The destructor of the abstract base class */
    virtual ~HeightData()
    {
    }
    
    /**
     *  Get the height at a distinct point, in local coordinates, unit is ft
     *
     *  \param x_north  x coordinate (x positive == north)
     *  \param y_east   y coordinate (y positive == east)
     *
     *  \return terrain height at this point in ft
     */
    virtual float getHeight(float x_north, float y_east) = 0;

    /**
     *  Get height and plane equation at x|y, in local coordinates, unit is ft
     *
     *  \param x_north  x coordinate (x positive == north)
     *  \param y_east   y coordinate (y positive == east)
     *  \param tplane this is where the plane equation will be stored
     *  \return terrain height at this point in ft
     */
    virtual float getHeightAndPlane(float x_north, float y_east, float tplane[4]) = 0;

    float getHeightAndPlane_(float x_north, float y_east, float tplane[4]);
  
  private:
    ssgRoot* SceneGraph_;
};

#endif //HEIGHTDATA_H
