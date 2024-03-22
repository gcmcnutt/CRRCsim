/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 *   Copyright (C) 2000, 2001 Jan Kansky (original author)
 *   Copyright (C) 2004-2010 Jan Reucker
 *   Copyright (C) 2005, 2008 Jens Wilhelm Wulf
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

#ifndef MODEL_BASED_SCENERY_H
#define MODEL_BASED_SCENERY_H

#include <crrc_config.h>

#include "crrc_scenery.h"
#include "../mod_math/vector3.h"
#include "../mod_misc/SimpleXMLTransfer.h"
#include <plib/ssg.h>
#include "winddata3D.h"
#include "heightdata.h"

#define DEFAULT_HEIGHT_MODE   2


/**
 *  \brief Class for 3D-model-based sceneries
 *
 */
class ModelBasedScenery : public Scenery
{
  public:
    /**
     *  The constructor
     *
     *  \param xml SimpleXMLTransfer from which the base classes will be initialized
     */
    ModelBasedScenery(SimpleXMLTransfer *xml, int sky_variant);
  
    /**
     *  The destructor
     */
    ~ModelBasedScenery();
  
    /**
     *  Draw the scenery
     */
    void draw(double current_time);

    /**
     *  Draw the shadows casted by scenery's objects
     */
    void draw_shadows(double current_time);

    /**
     *  Get the height at a distinct point.
     *  \param x x coordinate
     *  \param z z coordinate
     *  \return terrain height at this point in ft
     */
    float getHeight(float x, float z);

    /**
     *  get height and plane equation at x|z
     *  \param x x coordinate
     *  \param z z coordinate
     *  \param tplane this is where the plane equation will be stored
     *  \return terrain height at this point in ft
     */
    float getHeightAndPlane(float x, float z, float tplane[4]);
    
    /**
     *  Get an ID code for this location or scenery type
     */
    int getID() {return location;};

    /**
     *  Get wind components at position X_cg, Y_cg, Z_cg
     */
    int getWindComponents(double X_cg, double Y_cg, double Z_cg,
                          float *x_wind_velocity, 
                          float *y_wind_velocity,
                          float *z_wind_velocity);
  
  private:
    ssgRoot        *SceneGraph;
    int location;   ///< location id
    int getHeight_mode;
      //0 : use ssgLOS (slow if many triangle)
      //1 : ssgLOS()s en table (not god)
      //2 : tiling of surface 

    HeightData *heightdata;

    void evaluateNodeAttributes(ssgEntity* ent, bool def_t, bool def_v, bool def_a);
    void addInstance(SimpleXMLTransfer *xml, bool c, bool s, ssgEntity *model, ssgBranch *parent);
    void addPopulation(SimpleXMLTransfer *xml, bool c, bool s, ssgEntity *model, ssgBranch *parent);
    void addObject(bool c, bool s, ssgEntity *model, ssgBranch *parent, sgMat4 xform);
    void activateSceneTree(ssgEntity *ent, bool fObject);
    void updateSceneShadows(ssgEntity *ent);
   
#if WINDDATA3D == 1
    int init_wind_data(const char* filename);
    int find_wind_data(float n, float e, float u, float *vx, float *vy, float * vz);
    WindData *wind_data;
    float wind_position_coef;
#endif
};

#endif  // MODEL_BASED_SCENERY_H
