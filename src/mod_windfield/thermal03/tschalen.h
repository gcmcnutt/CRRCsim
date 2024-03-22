/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2006, 2008 Jens Wilhelm Wulf (original author)
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

#ifndef TSCHALEN_H
#define TSCHALEN_H

#include <vector>
#include <iostream>
#include "../../mod_misc/SimpleXMLTransfer.h"
#include "thermikschale.h"

/**
 * External coordinate system of ThermikSchalen is:
 * y=0 at bottom of thermal, y=1 at top of thermal.
 * It is symmetric about its y-axis at y=0, so x is a radius.
 * 
 * @author Jens W. Wulf
 */
class ThermikSchalen
{
  public:
   /**
    * Init from XML description
    */
   void init(SimpleXMLTransfer* cfg);
   
   /**
    * Get velocity vector at (x|y).
    * vRef is reference velocity at center of thermal and y=0.5.
    */
   void vectorAt(flttype  x,  flttype  y,
                 flttype& dx, flttype& dy,
                 flttype  vRef);
      
   
   /**
    * Radius of thermal (including downstream) in external coordinates.
    */
   inline flttype get_r_max() { return(r_max); };
   
   /**
    * Reference radius in external coordinates.
    * This is the radius of the upstream at y=0.5.
    */
   inline flttype get_r_ref() { return(r_ref); };

   void createDefaultConfig(SimpleXMLTransfer* cfg);   
   void trans(flttype& x, flttype& y);
   void trans_re(flttype& x, flttype& y);
   void zeigPunkt(flttype x, flttype y, std::ostream& out = std::cout);
      
  private:
   
   /**
    * Maxmimum radius in external coordinates (from real center of thermal)
    */
   flttype r_max;
   
   /**
    * Reference radius in external coordinates
    */
   flttype r_ref;

   /**
    * Describes upwards velocity profile at average height.
    * 1 lets velocity rise linearily; a value of 2 makes sense. 
    * Bigger numbers result in a more 'blocky' profile.
    */
   flttype vRefExp;

   ThermikSchale inner;
   ThermikSchale outer;
   
   flttype x0;
   flttype flTransSin;
   flttype flTransCos;
   flttype flTransReSin;
   flttype flTransReCos;      
};

#endif
