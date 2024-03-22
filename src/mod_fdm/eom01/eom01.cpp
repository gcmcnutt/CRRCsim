// -*- mode: c; mode: fold -*-
/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2006, 2008, 2009 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2006, 2007, 2008 - Jan Reucker
 *   Copyright (C) 2006 - Todd Templeton
 *
 * This file is partially based on work by
 *   Jan Kansky
 *   Bruce Jackson
 * The respective methods have a header containing more details, see below.
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
#include "eom01.h"

#include <math.h>
#include <iostream>

#include "../../mod_misc/ls_constants.h"
#include "../ls_geodesy.h"

// A bigger value enables more details/effects. Note that a value of zero is 
// sufficiently realistic for an RC sim.
#define EOM_DETAIL       0
#define EOM_CURVED_EARTH 1

EOM01::EOM01(const char* logfilename, FDMEnviroment* myEnv) : FDMBase(logfilename, myEnv)
{
}

double EOM01::getPhi()
{
  return(euler_angles_v[0]);
}

double EOM01::getTheta()
{
  return(euler_angles_v[1]);
}

double EOM01::getPsi()
{
  return(euler_angles_v[2]);
}

CRRCMath::Vector3 EOM01::getPos()
{
  return(v_P_CG_Rwy);
}

CRRCMath::Vector3 EOM01::getVel()
{
  return(v_V_local_rel_ground);
}

CRRCMath::Vector3 EOM01::getAccel()
{
  return(v_V_dot_local);
}

CRRCMath::Vector3 EOM01::getPQR()
{
  return(v_R_omega_body);
}

double EOM01::getLat()
{
  return(Latitude);
}

double EOM01::getLon()
{
  return(Longitude);
}

double EOM01::getAlt()
{
  return(Altitude);
}

void EOM01::ls_step_init() 
{
  /* Set past values to zero */
  v_V_dot_past = CRRCMath::Vector3();
  latitude_dot_past = longitude_dot_past = radius_dot_past  = 0;
  v_R_omega_dot_body_past = CRRCMath::Vector3();
  e_dot_0_past = e_dot_1_past = e_dot_2_past = e_dot_3_past = 0;

  /* Initialize geocentric position from geodetic latitude and altitude */

  ls_geod_to_geoc( Latitude, Altitude, &Sea_level_radius, &Lat_geocentric);
  Lon_geocentric = Longitude;
  Radius_to_vehicle = Altitude + Sea_level_radius;

  /* Initialize quaternions and transformation matrix from Euler angles */

  e_0 = cos(Psi*0.5)*cos(Theta*0.5)*cos(Phi*0.5)
    + sin(Psi*0.5)*sin(Theta*0.5)*sin(Phi*0.5);
  e_1 = cos(Psi*0.5)*cos(Theta*0.5)*sin(Phi*0.5)
    - sin(Psi*0.5)*sin(Theta*0.5)*cos(Phi*0.5);
  e_2 = cos(Psi*0.5)*sin(Theta*0.5)*cos(Phi*0.5)
    + sin(Psi*0.5)*cos(Theta*0.5)*sin(Phi*0.5);
  e_3 = -cos(Psi*0.5)*sin(Theta*0.5)*sin(Phi*0.5)
    + sin(Psi*0.5)*cos(Theta*0.5)*cos(Phi*0.5);
  LocalToBody.v[0][0] = e_0*e_0 + e_1*e_1 - e_2*e_2 - e_3*e_3;
  LocalToBody.v[0][1] = 2*(e_1*e_2 + e_0*e_3);
  LocalToBody.v[0][2] = 2*(e_1*e_3 - e_0*e_2);
  LocalToBody.v[1][0] = 2*(e_1*e_2 - e_0*e_3);
  LocalToBody.v[1][1] = e_0*e_0 - e_1*e_1 + e_2*e_2 - e_3*e_3;
  LocalToBody.v[1][2] = 2*(e_2*e_3 + e_0*e_1);
  LocalToBody.v[2][0] = 2*(e_1*e_3 + e_0*e_2);
  LocalToBody.v[2][1] = 2*(e_2*e_3 - e_0*e_1);
  LocalToBody.v[2][2] = e_0*e_0 - e_1*e_1 - e_2*e_2 + e_3*e_3;  
}


/**
 * This code is based on ls_step.c in the original version of CRRCSim, which said:
 * 
 *                      Written 920802 by Bruce Jackson.  Based upon equations
 *                      given in reference [1] and a Matrix-X/System Build block
 *                      diagram model of equations of motion coded by David Raney
 *                      at NASA-Langley in June of 1992.    
 *  
 *              [ 1]    McFarland, Richard E.: "A Standard Kinematic Model
 *                      for Flight Simulation at NASA-Ames", NASA CR-2497,
 *                      January 1975
 *  
 *              [ 2]    ANSI/AIAA R-004-1992 "Recommended Practice: Atmos-
 *                      pheric and Space Flight Vehicle Coordinate Systems",
 *                      February 1992
 */
void EOM01::ls_step( SCALAR dt )   
{
  SCALAR        dth;
  SCALAR        epsilon, inv_eps;
  SCALAR        e_dot_0, e_dot_1, e_dot_2, e_dot_3;
  SCALAR        cos_Lat_geocentric, inv_Radius_to_vehicle;
    
  CRRCMath::Vector3    v_R_omega_total;    /* Diff btw B & L       */

  VECTOR_3    geocentric_rates_v;       /* Geocentric linear velocities */
#define Geocentric_rates_v      geocentric_rates_v
#define Latitude_dot            geocentric_rates_v[0]
#define Longitude_dot           geocentric_rates_v[1]
#define Radius_dot              geocentric_rates_v[2]

/* Update time */

  dth = 0.5*dt;

/*  L I N E A R   V E L O C I T I E S   */

/* Integrate linear accelerations to get velocities */
/*    Using predictive Adams-Bashford algorithm     */

  v_V_local += (v_V_dot_local*3 - v_V_dot_past)*dth;

/* record past states */

  v_V_dot_past = v_V_dot_local;

/* Calculate trajectory rate (geocentric coordinates) */

  inv_Radius_to_vehicle = 1.0/Radius_to_vehicle;
  cos_Lat_geocentric = cos(Lat_geocentric);

  if ( cos_Lat_geocentric != 0)
  {
    Longitude_dot = v_V_local.r[1]/(Radius_to_vehicle*cos_Lat_geocentric);
  }
  else
  {
    // This is just to stop some compilers from complaining about a
    // non-initialized Longitude_dot. It's not mathematically correct
    // (Longitude_dot will move towards +inf if the cosine gets 0),
    // but it also should be irrelevant and at least it's better than
    // relying on something that isn't initialized.
    Longitude_dot = 0;
    fprintf(stderr, "Error: Longitude_dot --> +inf!\n");
  }

  Latitude_dot = v_V_local.r[0]*inv_Radius_to_vehicle;
  Radius_dot   = -v_V_local.r[2];

/*  A N G U L A R   V E L O C I T I E S   A N D   P O S I T I O N S  */

/* Integrate rotational accelerations to get velocities */

  v_R_omega_body = v_R_omega_body + (v_R_omega_dot_body*3 - v_R_omega_dot_body_past)*dth;
  
  // sanity check: v_R_omega_body.length() * dt < pi/2
  {
    double vRo_max = 0.5*M_PI / dt;
    double vRo_len = v_R_omega_body.length();
    
    if (vRo_len > vRo_max)
      v_R_omega_body *= vRo_max/vRo_len;
  }
  
/* Save past states */
  
  v_R_omega_dot_body_past = v_R_omega_dot_body;

  if (EOM_DETAIL >= EOM_CURVED_EARTH)
  {
    CRRCMath::Vector3    v_R_omega_local;    /* Angular L rates      */
    CRRCMath::Vector3    v_R_local_in_body;
    
    /* Calculate local axis frame rates due to travel over curved earth */
    v_R_omega_local.r[0] =  v_V_local.r[1]*inv_Radius_to_vehicle;
    v_R_omega_local.r[1] = -v_V_local.r[0]*inv_Radius_to_vehicle;
    v_R_omega_local.r[2] = -v_V_local.r[1]*tan(Lat_geocentric)*inv_Radius_to_vehicle;
  
    /* Transform local axis frame rates to body axis rates */
    v_R_local_in_body = LocalToBody * v_R_omega_local;

    /* Calculate total angular rates in body axis */
    v_R_omega_total = v_R_omega_body - v_R_local_in_body;
  }
  else
    v_R_omega_total = v_R_omega_body;

/* Transform to quaternion rates (see Appendix E in [2]) */

  e_dot_0 = 0.5*( -v_R_omega_total.r[0]*e_1 - v_R_omega_total.r[1]*e_2 - v_R_omega_total.r[2]*e_3 );
  e_dot_1 = 0.5*(  v_R_omega_total.r[0]*e_0 - v_R_omega_total.r[1]*e_3 + v_R_omega_total.r[2]*e_2 );
  e_dot_2 = 0.5*(  v_R_omega_total.r[0]*e_3 + v_R_omega_total.r[1]*e_0 - v_R_omega_total.r[2]*e_1 );
  e_dot_3 = 0.5*( -v_R_omega_total.r[0]*e_2 + v_R_omega_total.r[1]*e_1 + v_R_omega_total.r[2]*e_0 );

/* Integrate using trapezoidal as before */

  e_0 = e_0 + dth*(e_dot_0 + e_dot_0_past);
  e_1 = e_1 + dth*(e_dot_1 + e_dot_1_past);
  e_2 = e_2 + dth*(e_dot_2 + e_dot_2_past);
  e_3 = e_3 + dth*(e_dot_3 + e_dot_3_past);

/* calculate orthagonality correction  - scale quaternion to unity length */

  epsilon = sqrt(e_0*e_0 + e_1*e_1 + e_2*e_2 + e_3*e_3);
  inv_eps = 1/epsilon;

  e_0 = inv_eps*e_0;
  e_1 = inv_eps*e_1;
  e_2 = inv_eps*e_2;
  e_3 = inv_eps*e_3;

/* Save past values */

  e_dot_0_past = e_dot_0;
  e_dot_1_past = e_dot_1;
  e_dot_2_past = e_dot_2;
  e_dot_3_past = e_dot_3;

/* Update local to body transformation matrix */

  LocalToBody.v[0][0] = e_0*e_0 + e_1*e_1 - e_2*e_2 - e_3*e_3;
  LocalToBody.v[0][1] = 2*(e_1*e_2 + e_0*e_3);
  LocalToBody.v[0][2] = 2*(e_1*e_3 - e_0*e_2);
  LocalToBody.v[1][0] = 2*(e_1*e_2 - e_0*e_3);
  LocalToBody.v[1][1] = e_0*e_0 - e_1*e_1 + e_2*e_2 - e_3*e_3;
  LocalToBody.v[1][2] = 2*(e_2*e_3 + e_0*e_1);
  LocalToBody.v[2][0] = 2*(e_1*e_3 + e_0*e_2);
  LocalToBody.v[2][1] = 2*(e_2*e_3 - e_0*e_1);
  LocalToBody.v[2][2] = e_0*e_0 - e_1*e_1 - e_2*e_2 + e_3*e_3;
  
/* Calculate Euler angles */

  Theta = asin( -1*LocalToBody.v[0][2] );

  if( LocalToBody.v[0][0] == 0 )
    Psi = 0;
  else
    Psi = atan2( LocalToBody.v[0][1], LocalToBody.v[0][0] );

  if( LocalToBody.v[2][2] == 0 )
    Phi = 0;
  else
    Phi = atan2( LocalToBody.v[1][2], LocalToBody.v[2][2] );

/* Resolve Psi to 0 - 359.9999 */

  if (Psi < 0 ) Psi = Psi + 2*M_PI;

/*  L I N E A R   P O S I T I O N S   */

/* Trapezoidal acceleration for position */

  Lat_geocentric       = Lat_geocentric    + dth*(Latitude_dot  + latitude_dot_past );
  Lon_geocentric       = Lon_geocentric    + dth*(Longitude_dot + longitude_dot_past);
  Radius_to_vehicle    = Radius_to_vehicle + dth*(Radius_dot    + radius_dot_past );

/* Save past values */

  latitude_dot_past  = Latitude_dot;
  longitude_dot_past = Longitude_dot;
  radius_dot_past    = Radius_dot;

/* end of ls_step */
}


/**
 * This method is largely based on ls_aux.c (initial version of CRRCSim), which is 
 * taken from LaRCSIM and was created 9208026 as part of C-castle simulation project
 * by Bruce Jackson.
 */
void EOM01::ls_aux(CRRCMath::Vector3 v_V_local_airmass, CRRCMath::Vector3 v_V_gust_body)
{
  /* update geodetic position */
  ls_geoc_to_geod_fastbowring(Lat_geocentric, Radius_to_vehicle,
                              &Latitude, &Altitude, &Sea_level_radius);

  Longitude = Lon_geocentric;

  /* Form relative velocity vector */

  v_V_local_rel_ground = v_V_local;
  v_V_local_rel_airmass = v_V_local_rel_ground - v_V_local_airmass;
  v_V_wind_body = LocalToBody * v_V_local_rel_airmass - v_V_gust_body;
  
  V_rel_wind = v_V_wind_body.length();

  /* Calculate flight path and other flight condition values */

  if (v_V_wind_body.r[0] == 0)
    Alpha = 0;
  else
    Alpha = atan2( v_V_wind_body.r[2], v_V_wind_body.r[0] );

  if (V_rel_wind == 0)
    Beta = 0;
  else
    Beta = asin( v_V_wind_body.r[1]/ V_rel_wind );

  /* Calculate local gravity  */

  // original code called
  //      ls_gravity( Radius_to_vehicle, Lat_geocentric, &Gravity );
  Gravity = env->GetG(Altitude);
    
  /* call function for (smoothed) density ratio, sonic velocity, and ambient pressure */

  Density = env->GetRho(Altitude);
  
  /* Determine location in runway coordinates */

  v_P_CG_Rwy.r[0] = Sea_level_radius * Latitude;
  v_P_CG_Rwy.r[1] = Sea_level_radius * Longitude;
  v_P_CG_Rwy.r[2] = Sea_level_radius - Radius_to_vehicle;
  
  /* end of ls_aux */
}


/**
 * This code is based on ls_accel.c in the original version of CRRCSim, which said:
 * 
 *    Written 920731 by Bruce Jackson.  Based upon equations
 *    given in reference [1] and a Matrix-X/System Build block
 *    diagram model of equations of motion coded by David Raney
 *    at NASA-Langley in June of 1992.
 */
void EOM01::ls_accel(CRRCMath::Vector3 v_F,
                     CRRCMath::Vector3 v_M_cg,
                     float fixed_z,
                     bool  fFixedHorizon)
{
  SCALAR        inv_Mass, inv_Radius;
  SCALAR        ixz2, c0, c1, c2, c3, c4, c5, c6, c7, c8, c9, c10;
  SCALAR        tan_Lat_geocentric;

  CRRCMath::Vector3 v_F_local;
    
  if (isfinite(v_F.length()) == 0)
  {
    v_F = CRRCMath::Vector3();
    std::cerr << "warning: used BSOD workaround\n";
  }
  if (isfinite(v_M_cg.length()) == 0)
  {
    v_M_cg = CRRCMath::Vector3();
    std::cerr << "warning: used BSOD workaround\n";
  }
  
#if (EOM_TEST == 2)
  
  switch ((nLogCnt>>5) % 6)
  {
   case 0:
    v_F    = CRRCMath::Vector3(1, 0, 0);
    v_M_cg = CRRCMath::Vector3(0, 0, 0);
    break;
   case 1:
    v_F    = CRRCMath::Vector3(0, 0, 0);
    v_M_cg = CRRCMath::Vector3(0, 2, 0);
    break;
   case 2:
    v_F    = CRRCMath::Vector3(0, 0, 3);
    v_M_cg = CRRCMath::Vector3(0, 0, 0);
    break;
   case 3:
    v_F    = CRRCMath::Vector3(0, 0, 0);
    v_M_cg = CRRCMath::Vector3(1, 0, 0);
    break;
   case 4:
    v_F    = CRRCMath::Vector3(0, 2, 0);
    v_M_cg = CRRCMath::Vector3(0, 0, 0);
    break;
   case 5:
    v_F    = CRRCMath::Vector3(0, 0, 0);
    v_M_cg = CRRCMath::Vector3(0, 0, 3);
    break;
  }
  
  {
    double d = sin(M_PI*(nLogCnt&0x1F)/32.0);
    
    v_F    *= d*d;
    v_M_cg *= d*d;
  }
  
  
  if (  ((nLogCnt>>5)/6) % 2)
  {
    v_F    *= -1;
    v_M_cg *= -1;
  }
  
#endif  
  
  /* Transform from body to local frame */

  v_F_local = LocalToBody.multrans(v_F);
  
  /* Calculate linear accelerations */

  tan_Lat_geocentric = tan(Lat_geocentric);
  inv_Mass    = 1/Mass;
  inv_Radius  = 1/Radius_to_vehicle;
  
  v_V_dot_local.r[0] = inv_Mass*v_F_local.r[0] + inv_Radius*(v_V_local.r[0]*v_V_local.r[2] - v_V_local.r[1]*v_V_local.r[1] *tan_Lat_geocentric);
  v_V_dot_local.r[1] = inv_Mass*v_F_local.r[1] + inv_Radius*(v_V_local.r[1]*v_V_local.r[2]  + v_V_local.r[0]*v_V_local.r[1]*tan_Lat_geocentric);
#if EOM_TEST != 0
  v_V_dot_local.r[2] = inv_Mass*v_F_local.r[2]           - inv_Radius*(v_V_local.r[0]*v_V_local.r[0] + v_V_local.r[1]*v_V_local.r[1]);
#else
  v_V_dot_local.r[2] = inv_Mass*v_F_local.r[2] + Gravity - inv_Radius*(v_V_local.r[0]*v_V_local.r[0] + v_V_local.r[1]*v_V_local.r[1]);
#endif
  
  // The altitude-controller, because it is very easy here:
  if (fixed_z < EOM01_FIXED_Z_OFF*0.98)
  {
    v_V_dot_local.r[2] = Controller_s(fixed_z + Altitude, v_V_local.r[2]);
  }  
  
  /* Invert the symmetric inertia matrix */
  ixz2 = I_xz*I_xz;
  c0  = 1/(I_xx*I_zz - ixz2);
  c1  = c0*((I_yy-I_zz)*I_zz - ixz2);
  c4  = c0*I_xz;
  /* c2  = c0*I_xz*(I_xx - I_yy + I_zz); */
  c2  = c4*(I_xx - I_yy + I_zz);
  c3  = c0*I_zz;
  c7  = 1/I_yy;
  c5  = c7*(I_zz - I_xx);
  c6  = c7*I_xz;
  c8  = c0*((I_xx - I_yy)*I_xx + ixz2);
  /* c9  = c0*I_xz*(I_yy - I_zz - I_xx); */
  c9  = c4*(I_yy - I_zz - I_xx);
  c10 = c0*I_xx;

  /* Calculate the rotational body axis accelerations */

  v_R_omega_dot_body.r[0] = (c1*v_R_omega_body.r[2] + c2*v_R_omega_body.r[0])*v_R_omega_body.r[1] + c3*v_M_cg.r[0] +  c4*v_M_cg.r[2];
  v_R_omega_dot_body.r[1] =  c5*v_R_omega_body.r[2]*v_R_omega_body.r[0] + c6*(v_R_omega_body.r[2]*v_R_omega_body.r[2] - v_R_omega_body.r[0]*v_R_omega_body.r[0]) + c7*v_M_cg.r[1];
  v_R_omega_dot_body.r[2] = (c8*v_R_omega_body.r[0] + c9*v_R_omega_body.r[2])*v_R_omega_body.r[1] + c4*v_M_cg.r[0] + c10*v_M_cg.r[2];

  // fixed horizon controller
  // There currently is a problem with this one: once one has hit ground hard, yaw axis is dead ?!?!
  // Everything is fine after reset again.
  if (fFixedHorizon)
  {
    v_R_omega_dot_body.r[0] = Controller_s(0 - Phi,   v_R_omega_body.r[0]);
    v_R_omega_dot_body.r[1] = Controller_s(0 - Theta, v_R_omega_body.r[1]);
  }
}

float EOM01::Controller_s(float s_diff, float v)
{
  // a = a_max
  // s = integral(v, t) = integral(t*a_max, t) = t^2 / 2 * a_max
  // t = sqrt(2*s/a_max)
  const float v_max = 55 * M_TO_FT;
  float a_max = 9.81 * M_TO_FT;
  
  float t = sqrt(2*fabs(s_diff)/a_max);
  float v_setp;
  
  if (t < 0.2)
    a_max *= 0.1;
  
  v_setp = sqrt(2*fabs(s_diff)/a_max) * a_max;
  
  if (s_diff > 0)
  {
    if (v_setp > v_max)
      v_setp = v_max;      
  }
  else
  {
    v_setp = -v_setp;
    if (v_setp < -v_max)
      v_setp = -v_max;
  }
  
  // I don't know dt here, but I assume something save: 20ms
  return(v_setp - v)/20.0E-3;
}

CRRCMath::Vector3 EOM01::WorldToBody(CRRCMath::Vector3 vWorld)
{
  return( LocalToBody * vWorld );
}
