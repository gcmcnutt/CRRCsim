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

#ifndef THERMALSOLVE_H
#define THERMALSOLVE_H

#include "thconf.h"

/**
 * 
 * This has initially been a general solver for fourth order equations.
 * Now it is specialized.
 * 
 * @author Jens W. Wulf
 */
class SolveFourthOrder
{
  public:
   flttype n0;
   flttype n1;
   flttype n2;
   flttype n3;
   flttype n4;

   flttype res;

   flttype start[5];
   flttype xi[5];
   flttype yi[5];
   unsigned int     anz;
      
   static flttype      precmin;
   static unsigned int loopmax;
   
   flttype ell_p1;
   flttype ell_p2;
   flttype ell_p3;
   flttype ell_p4;
   flttype ell_p5;
   flttype ell_x;
   flttype ell_y;
   
   /**
    * Sucht nur nach Lösungen im Bereich 0..1
    */
   void solve();
};

#endif
