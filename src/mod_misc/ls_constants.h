/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 1993 Bruce Jackson (original author)
 * Copyright (C) 2000 Jan Edward Kansky
 * Copyright (C) 2005, 2006, 2008 Jens Wilhelm Wulf
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
  
/**
 * \file ls_constants.h
 *
 * LaRCSim constants definition header file
 *
 * REFERENCES:
 *
 * [ 7]    Anon: "Aeronautical Vest Pocket Handbook, 17th edition",
 *                Pratt & Whitney Aircraft Group, Dec. 1977
 */

#ifndef _LS_CONSTANTS
#define _LS_CONSTANTS

#ifndef M_PI
# define M_PI 3.14159265358979323846
#endif

/* Define constants (note: many factors will need to change for other 
        systems of measure)     */

/* miscellaneous units conversions (ref [7]) */
#define DEG_TO_RAD           (M_PI/180.0)
#define RAD_TO_DEG           (180.0/M_PI)
#define K_TO_R               (1.8)
#define R_TO_K               (0.55555556)
#define NSM_TO_PSF           (0.02088547)
#define PSF_TO_NSM           (47.8801826)
#define KCM_TO_SCF           (0.00194106)
#define SCF_TO_KCM           (515.183616)
#define LBF_TO_N             (4.44822)
#define N_TO_LBF             (1.0/LBF_TO_N)
#define NM_TO_LBFFT          (1.0/1.355818)
#define FT_TO_M              (0.3048)
#define M_TO_FT              (1.0/FT_TO_M)
#define SLUG_TO_KG           (14.5939041995)
#define KG_TO_SLUG           (1.0/SLUG_TO_KG)
#define KG_M_M_TO_SLUG_FT_FT (1.0/(SLUG_TO_KG*FT_TO_M*FT_TO_M))


#endif /* _LS_CONSTANTS_H */


/*------------------------- end of ls_constants.h -------------------------*/
