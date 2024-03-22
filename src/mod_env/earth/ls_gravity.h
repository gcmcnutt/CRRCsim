/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2000, 2003 - Jan Edward Kansky (original author)
 *   Copyright (C) 2008 - Jens Wilhelm Wulf
 *   Copyright (C) 2005 - Lionel Cailler
 *
 * This program is free software; you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation; either version 2 of the License, or     
 * (at your option) any later version.                                   
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

#ifndef _LS_GRAVITY_H
#define _LS_GRAVITY_H

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

void ls_gravity( double radius, double lat, double *gravity );

double ls_gravity_g(double altitude);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _LS_GRAVITY_H */
