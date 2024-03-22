// -*- mode: c; mode: fold -*-
/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005 - 2009 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2006, 2008 - Jan Reucker
 *   Copyright (C) 2006 - Todd Templeton
 * 
 * This file is partially based on work by
 *   Jan Kansky
 *   Bruce Jackson
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
//
#ifndef EOM01_H
# define EOM01_H

# include <vector>
# include "../ls_types.h"
# include "../fdm.h"
# include "../../mod_math/vector3.h"
# include "../../mod_math/matrix33.h"

/**
 * equations of motion
 * 
 * References originally given by Bruce Jackson:
 * 
 *                [ 1]    McFarland, Richard E.: "A Standard Kinematic Model
 *                        for Flight Simulation at NASA-Ames", NASA CR-2497,
 *                        January 1975
 *                [ 2]    ANSI/AIAA R-004-1992 "Recommended Practice: Atmos-
 *                        pheric and Space Flight Vehicle Coordinate Systems",
 *                        February 1992
 *                [ 3]    Beyer, William H., editor: "CRC Standard Mathematical
 *                        Tables, 28th edition", CRC Press, Boca Raton, FL, 1987,
 *                        ISBN 0-8493-0628-0
 *                [ 4]    Dowdy, M. C.; Jackson, E. B.; and Nichols, J. H.:
 *                        "Controls Analysis and Simulation Test Loop Environ-
 *                        ment (CASTLE) Programmer's Guide, Version 1.3",
 *                        NATC TM 89-11, 30 March 1989.
 *                [ 5]    Halliday, David; and Resnick, Robert: "Fundamentals
 *                        of Physics, Revised Printing", Wiley and Sons, 1974.
 *                        ISBN 0-471-34431-1
 *                [ 6]    Anon: "U. S. Standard Atmosphere, 1962"
 *                [ 7]    Anon: "Aeronautical Vest Pocket Handbook, 17th edition",
 *                        Pratt & Whitney Aircraft Group, Dec. 1977
 *                [ 8]    Stevens, Brian L.; and Lewis, Frank L.: "Aircraft
 *                        Control and Simulation", Wiley and Sons, 1992.
 *                        ISBN 0-471-61397-5
 * 
 * @author Bruce Jackson
 * @author Jens Wilhelm Wulf
 */
class EOM01 : public FDMBase
{   
public:
  
  /**
   * The world coordinate vector vWorld is transformed
   * to body coordinates and returned.
   */
  virtual CRRCMath::Vector3 WorldToBody(CRRCMath::Vector3 vWorld);
  
  virtual CRRCMath::Vector3 getPos();
  virtual double getPhi();
  virtual double getTheta();
  virtual double getPsi();
  
  /**
   * returns velocity w.r.t. earth surface
   */
  virtual CRRCMath::Vector3 getVel();
  virtual CRRCMath::Vector3 getAccel();
  
  /**
   * omega vector (rotational velocity). positive values:
   *   p right wing down
   *   q nose up
   *   r nose right
   */
  virtual CRRCMath::Vector3 getPQR();
  
  virtual double getLat();
  virtual double getLon();
  virtual double getAlt();
  
  /**
   * Returns velocity relative to airmass [ft/s].
   */
  virtual double getVRelAirmass() { return(V_rel_wind); };
  
  /**
   * Parameters are simply handed over to FDMBase
   */
  EOM01(const char* logfilename, FDMEnviroment* myEnv);

protected:
  
  /// @name Mass and inertia
  //@{
  SCALAR Mass;  // inertia
  SCALAR I_xx;  // inertia
  SCALAR I_yy;  // inertia
  SCALAR I_zz;  // inertia
  SCALAR I_xz;  // inertia
  //@}         
  
  /**
   * v_V_local_airmass: velocity of airmass (steady winds), north, east, down, ft/s
   * v_V_gust_body:     linear turbulence components, body frame
   */
  void ls_aux(CRRCMath::Vector3 v_V_local_airmass, CRRCMath::Vector3 v_V_gust_body);
  
  virtual void ls_step_init();
  
  void ls_step( SCALAR dt);
  
  /**
   * if fixed_z < EOM01_FIXED_Z_OFF, this altitude is forced by a controller
   */
  void ls_accel(CRRCMath::Vector3 v_F,
                CRRCMath::Vector3 v_M_cg,
                float             fixed_z       = EOM01_FIXED_Z_OFF,
                bool              fFixedHorizon = false);

  float Controller_s(float s_diff, float v);

protected:
  
  /// @name written by step
  //@{
  SCALAR  latitude_dot_past, longitude_dot_past, radius_dot_past;
  
  /**
   * P, Q, R
   */
  CRRCMath::Vector3 v_R_omega_dot_body_past;
  
  /**
   * north, east, down
   */
  CRRCMath::Vector3 v_V_dot_past;
  
  SCALAR  e_0, e_1, e_2, e_3;
  SCALAR  e_dot_0_past, e_dot_1_past, e_dot_2_past, e_dot_3_past;
    
  /**
   * Transformation matrix local to body
   */
  CRRCMath::Matrix33 LocalToBody;
  
  /**
   * Angular body rates
   */
  CRRCMath::Vector3 v_R_omega_body;
  
  CRRCMath::Vector3 v_V_local;
  
  VECTOR_3    euler_angles_v;
# define Euler_angles_v   euler_angles_v
# define Phi              euler_angles_v[0]
# define Theta            euler_angles_v[1]
# define Psi              euler_angles_v[2]
  
  VECTOR_3    geocentric_position_v;
# define Geocentric_position_v  geocentric_position_v
# define Lat_geocentric         geocentric_position_v[0]
# define Lon_geocentric         geocentric_position_v[1]
# define Radius_to_vehicle      geocentric_position_v[2]
  //@}   
  
  /// @name written by accel
  //@{

  CRRCMath::Vector3 v_R_omega_dot_body;
  
  CRRCMath::Vector3 v_V_dot_local;
  
  //@}
  
  /// @name written by aux
  //@{

  SCALAR Sea_level_radius;
  
  VECTOR_3 geodetic_position_v;
# define Geodetic_position_v  geodetic_position_v
# define Latitude             geodetic_position_v[0]
# define Longitude            geodetic_position_v[1]
# define Altitude             geodetic_position_v[2]  
  
  /**
   * V rel w.r.t. earth surface
   */
  CRRCMath::Vector3 v_V_local_rel_ground;
  
  /**
   * Wind-relative velocities  
   */
  CRRCMath::Vector3 v_V_local_rel_airmass;

  /**
   * Wind-relative velocities in body axis  
   */
  CRRCMath::Vector3 v_V_wind_body;
  
  SCALAR    V_rel_wind;
  SCALAR    Alpha, Beta;  /* in radians */
  SCALAR    Gravity;      /* Local acceleration due to G */
  SCALAR    Density;
  
  /**
   * CG relative to runway, in rwy coordinates
   * north/east/down
   */
  CRRCMath::Vector3 v_P_CG_Rwy;
  
  //@}
};

#endif
