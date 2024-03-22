/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2000, 2001 Jan Edward Kansky (original author)
 * Copyright (C) 2005, 2006, 2008 Jens Wilhelm Wulf
 * Copyright (C) 2005, 2007 Jan Reucker
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
  

#ifndef WINDFIELD_H
#define WINDFIELD_H

#include "../mod_windfield_config.h"
#include "../mod_math/vector3.h"
#include "../mod_math/matrix33.h"
#include "../mod_misc/SimpleXMLTransfer.h"
#include "../mod_misc/crrc_rand.h"


class ThermikSchalen;

/** \brief A class that represents a thermal
 *
 *  This class replaces the old "thermal" data struct.
 */
class Thermal
//typedef struct
{
  private:
    int xcoord;               ///< X coordinate in thermal occupancy grid
    int ycoord;               ///< Y coordinate in thermal occupancy grid
    bool fInvisible;          ///< thermal is not visible in grid

  public:
    Thermal *next_thermal;
    float center_x_position;  ///< Center position of thermal on ground
    float center_y_position;  ///< Center position of thermal on ground
    float radius;             ///< Radius of thermal column ft
  #if (THERMAL_CODE == 0)
    float boundary_thickness; ///< 1/e width of transition into thermal core
  #endif
    float strength;           ///< Vertical component strength in ft/s
    float lifetime;           ///< remaining lifetime in sec

    /// create a thermal and add its graphical representation to the scene graph
    Thermal();

    /// initialize thermal with some sensible random values
    void random_init();
  
    /// remove thermal from thermal grid
    void remove_from_grid();
  
    /// update the thermal
    void update(float flDeltaT, float x_motion, float y_motion);
    
    /**
     * Sums velocities of thermal at dX|dY|dZ.
     * Only for version 3.
     */
    void sumVelocity(double X_cg, double Y_cg, double Z_cg,
                     ThermikSchalen& thermalv3,
                     double& Vel_north, double& Vel_east, double& Vel_down);

    /**
     * Calculate vertical velocity of thermal at dX|dY|dZ.
     * Only for (THERMAL_CODE == 1).
     */    
    double getVelocity(double dX, double dY, double dZ);
    
    /// draw the thermal
    void draw(double H_cg_rwy);
};
//} Thermal;

/**
 * Initialize thermal positions stregths, radii, etc.
 */
void initialize_wind_field(SimpleXMLTransfer* el);

/**
 * Creates a description with default (v3) thermal settings
 */
SimpleXMLTransfer* GetDefaultConf_Thermal();

/**
 * removes everything from the wind field
 */
void clear_wind_field();

/**
 * Given the time since the last iteration, this function:
 * -moves thermals with the wind
 * -destroys thermals after their lifetime or when they leave the grid
 * -creates new thermals
 */
void update_thermals(float flDeltaT);

/**
 * Calculate the wind velocities in all three axes in the given position.
 * Returns 1 if this position is outside of the grid.
 * X/Y/Z -- north/east/down
 */
int calculate_wind(double  X_cg,      double  Y_cg,     double  Z_cg,
                   double& Vel_north, double& Vel_east, double& Vel_down);

/**
 * Calculate the gradient of wind velocity in the north/east/down frame.
 */
int calculate_wind_grad(double X_cg, double Y_cg, double Z_cg, double delta_space,
                        CRRCMath::Matrix33& m_V_grad);

/**
 * Static data for wind turbulence model
 */
static RandGauss eta1, eta2, eta3, eta4;
static CRRCMath::Vector3 v_V_gust_body_, v_V_gust_body_old_;
static CRRCMath::Vector3 v_R_omega_gust_body_;

/**
 * Initialize gust (a.k.a wind turbulence) linear and rotational
 * velocities in body axes.
 */
void initialize_gust();

/**
 * Given the time since last iteration updates gust linear
 * and rotational velocities in body axes.
 */
void calculate_gust(double dt, double altitude, double V_rel_wind, double b,
                    CRRCMath::Vector3 v_V_local_airmass,
                    CRRCMath::Matrix33 LocalToBody,
                    CRRCMath::Vector3& v_V_gust_body,
                    CRRCMath::Vector3& v_R_omega_gust_body);

/** \brief Draw the thermals.
 *
 *  Draws a sphere for each thermal within a given square around the aircraft.
 */
void draw_thermals(CRRCMath::Vector3 pos);

/** 
 *
 *  Draws an indicator for wind strength and direction
 */
void draw_wind(double direction_face);

/**
 * Maximum thermal density (1/ft^2)
 */
double getMaxThermalDensity();

#if DEBUG_THERMAL_SCRSHOT == 1
/**
 * Writes up/down velocity of the windfield to a file
 */
void windfield_thermalScreenshot(CRRCMath::Vector3 pos);

#endif

#endif
