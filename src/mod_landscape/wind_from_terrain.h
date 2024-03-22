/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 *  Copyright (C) 2010 Joel Lienard (original author)
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
  

/*
 * crrc_wind_from_terrain.h
 * very simple wind calculation from terain profil
 */
#ifndef CRRC_WINDFROMTERRAIN_H
#define CRRC_WINDFROMTERRAIN_H

 
int wind_from_terrain(double X, double Y, double Z,
    float *x_wind_velocity, float *y_wind_velocity, float *z_wind_velocity);
    
#endif //CRRC_WINDFROMTERRAIN_H

