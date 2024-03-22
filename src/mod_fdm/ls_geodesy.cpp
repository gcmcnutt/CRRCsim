/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) ? - Bruce Jackson (original author)
 *   Copyright (C) 2007, 2008 - Jan Reucker
 *   Copyright (C) 2008 - Jens Wilhelm Wulf
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

#include "../mod_misc/ls_constants.h"
#include "ls_geodesy.h"
#include <math.h>

/* ONE_SECOND is pi/180/60/60, or about 100 feet at earths' equator */
#define ONE_SECOND  4.848136811E-6
#define HALF_PI     (0.5*M_PI)

/* Value of earth radius from [1], ft */
#define EQUATORIAL_RADIUS 20925650.
#define RESQ 437882827922500.

/* Value of earth flattening parameter from ref [1]
	
	Note: FP = f
	      E  = 1-f
	      EPS = sqrt(1-(1-f)^2)			*/
	      
//#define FP	.003352813178         unused
#define E   .996647186
#define EPS .081819221
//#define INVG .031080997             unused

#define POLAR_RADIUS 20855490.2
#define MAX_ERROR 0.01
#define EPS2 0.006694385  // square of first eccentricity = EPS*EPS
#define EPSP2 0.006739502  // square of the second eccentricity = EPS2 / (1 - EPS2)


/**
 * Written as part of LaRCSim project by E. B. Jackson
 * 
 * [ 1]  Stevens, Brian L.; and Lewis, Frank L.: "Aircraft
 *       Control and Simulation", Wiley and Sons, 1992.
 *       ISBN 0-471-61397-5
 */
void ls_geoc_to_geod(double lat_geoc, double radius, double * lat_geod,
                     double * alt, double * sea_level_r)
{
  double t_lat, x_alpha, mu_alpha, delt_mu, r_alpha, l_point, rho_alpha;
  double sin_mu_a, denom, delt_lambda, lambda_sl, sin_lambda_sl;

  if (((HALF_PI - lat_geoc) < ONE_SECOND) /* near North pole */
      || ((HALF_PI + lat_geoc) < ONE_SECOND)) /* near South pole */
  {
    *lat_geod = lat_geoc;
    *sea_level_r = EQUATORIAL_RADIUS * E;
    *alt = radius - *sea_level_r;
  }
  else
  {
    t_lat = tan(lat_geoc);
    x_alpha = E * EQUATORIAL_RADIUS / sqrt(t_lat * t_lat + E * E);
    mu_alpha = atan2(sqrt(RESQ - x_alpha * x_alpha), E * x_alpha);
    if (lat_geoc < 0)
      mu_alpha = -mu_alpha;
    sin_mu_a = sin(mu_alpha);
    delt_lambda = mu_alpha - lat_geoc;
    r_alpha = x_alpha / cos(lat_geoc);
    l_point = radius - r_alpha;
    *alt = l_point * cos(delt_lambda);
    denom = sqrt(1 - EPS * EPS * sin_mu_a * sin_mu_a);
    rho_alpha = EQUATORIAL_RADIUS * (1 - EPS) / (denom * denom * denom);
    delt_mu = atan2(l_point * sin(delt_lambda), rho_alpha + *alt);
    *lat_geod = mu_alpha - delt_mu;
    lambda_sl = atan(E * E * tan(*lat_geod)); /* SL geoc. latitude */
    sin_lambda_sl = sin(lambda_sl);
    *sea_level_r = sqrt(RESQ
                        / (1 +
                           ((1 / (E * E)) -
                            1) * sin_lambda_sl * sin_lambda_sl));
  }
}


void ls_geod_to_geoc(double lat_geod, double alt,
                     double * sl_radius, double * lat_geoc)
{
  double lambda_sl, sin_lambda_sl, cos_lambda_sl, sin_mu, cos_mu, px, py;

  lambda_sl = atan(E * E * tan(lat_geod));  /* sea level geocentric latitude */
  sin_lambda_sl = sin(lambda_sl);
  cos_lambda_sl = cos(lambda_sl);
  sin_mu = sin(lat_geod);       /* Geodetic (map makers') latitude */
  cos_mu = cos(lat_geod);
  *sl_radius = sqrt(RESQ
                    / (1 +
                       ((1 / (E * E)) - 1) * sin_lambda_sl * sin_lambda_sl));
  py = *sl_radius * sin_lambda_sl + alt * sin_mu;
  px = *sl_radius * cos_lambda_sl + alt * cos_mu;
  *lat_geoc = atan2(py, px);
}

void ls_geoc_to_geod_fastbowring(double lat_geoc, double radius, double * lat_geod,
                                 double * alt, double * sea_level_r)
{
  double W, Z, T, U, S, A, B, C, C_new;
  double sin_phi, cos_phi, sin_beta, cos_beta, RN;
  double sea_level_x, sea_level_z;
  
  W = radius*cos(lat_geoc);
  Z = radius*sin(lat_geoc);

  // Initial values
  T = Z*EQUATORIAL_RADIUS;
  U = W*POLAR_RADIUS;
  C = 0;

  for(int i =0; i < 5; i++)
  {
    S = sqrt(T*T + U*U);
    
    sin_beta = T/S;
    cos_beta = U/S;

    A = Z + POLAR_RADIUS * EPSP2 * sin_beta*sin_beta*sin_beta;
    B = W - EQUATORIAL_RADIUS * EPS2 * cos_beta*cos_beta*cos_beta;
    C_new = sqrt(A*A + B*B);

    sin_phi = A/C_new;
    cos_phi = B/C_new;
    
    if(i > 0 && fabs(C_new - C) < MAX_ERROR)
      break;
      
    C = C_new;

    T = E*sin_phi;
    U = cos_phi; 
  }
  
  // Normal radius, distance from surface to Z axis along ellipsoid normal
  RN = EQUATORIAL_RADIUS / sqrt(1 - EPS2 * sin_phi*sin_phi);
  
  *alt = W * cos_phi + (Z + EPS2 * RN * sin_phi) * sin_phi - RN;
  *lat_geod = atan2(sin_phi, cos_phi);
  
  sea_level_x = RN*cos_phi;
  sea_level_z = RN*(1-EPS2)*sin_phi;
    
  *sea_level_r = sqrt(sea_level_x*sea_level_x + sea_level_z*sea_level_z);
}

