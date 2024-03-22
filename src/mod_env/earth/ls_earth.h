/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 1993 - Bruce Jackson (ls_constants.h; was part of old ls_eom.h header file)
 *   Copyright (C) 2008 - Jens Wilhelm Wulf
 *   Copyright (C) 2008 - Jan Reucker
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

  REFERENCES:
  
    [ 1]  McFarland, Richard E.: "A Standard Kinematic Model
          for Flight Simulation at NASA-Ames", NASA CR-2497,
          January 1975
      
    [ 2]  ANSI/AIAA R-004-1992 "Recommended Practice: Atmos-
          pheric and Space Flight Vehicle Coordinate Systems",
          February 1992
      
    [ 3]  Beyer, William H., editor: "CRC Standard Mathematical
          Tables, 28th edition", CRC Press, Boca Raton, FL, 1987,
          ISBN 0-8493-0628-0
      
    [ 4]  Dowdy, M. C.; Jackson, E. B.; and Nichols, J. H.:
          "Controls Analysis and Simulation Test Loop Environ-
          ment (CASTLE) Programmer's Guide, Version 1.3", 
          NATC TM 89-11, 30 March 1989.
      
    [ 5]  Halliday, David; and Resnick, Robert: "Fundamentals
          of Physics, Revised Printing", Wiley and Sons, 1974.
          ISBN 0-471-34431-1

    [ 6]  Anon: "U. S. Standard Atmosphere, 1962"
    
    [ 7]  Anon: "Aeronautical Vest Pocket Handbook, 17th edition",
          Pratt & Whitney Aircraft Group, Dec. 1977
      
    [ 8]  Stevens, Brian L.; and Lewis, Frank L.: "Aircraft 
          Control and Simulation", Wiley and Sons, 1992.
          ISBN 0-471-61397-5      

 --------------------------------------------------------------------------*/
#ifndef _LS_CONSTANTS_EARTH
# define _LS_CONSTANTS_EARTH

/* Value of earth radius from [8], ft */
#define  EQUATORIAL_RADIUS 20925650.

/* ENGLISH Atmospheric reference properties [6] */
#define SEA_LEVEL_DENSITY  0.002376888

#endif
