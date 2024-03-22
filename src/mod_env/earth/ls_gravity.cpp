/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 1992 - Bruce Jackson (gravity model for LaRCsim)
 *   Copyright (C) 2007 - Jan Reucker 
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
 * 
  REFERENCES: Stevens, Brian L.; and Lewis, Frank L.: "Aircraft
    Control and Simulation", Wiley and Sons, 1992.
    ISBN 0-471-

--------------------------------------------------------------------------*/
#include "ls_earth.h"
#include "ls_gravity.h"
#include <math.h>

#define GM 1.4076431E16
#define J2 1.08263E-3

double ls_gravity_g(double altitude)
{
  double g;
  ls_gravity(EQUATORIAL_RADIUS + altitude, 0, &g);
  
  return(g);
}

void ls_gravity( double radius, double lat, double *gravity )
{

  double radius_ratio, rrsq, sinsqlat;

  radius_ratio = radius/EQUATORIAL_RADIUS;
  rrsq = radius_ratio*radius_ratio;
  sinsqlat = sin(lat)*sin(lat);
  *gravity = (GM/(radius*radius))
             *sqrt(2.25*rrsq*rrsq*J2*J2*(5*sinsqlat*sinsqlat -2*sinsqlat + 1)
                   + 3*rrsq*J2*(1 - 3*sinsqlat) + 1);

}
