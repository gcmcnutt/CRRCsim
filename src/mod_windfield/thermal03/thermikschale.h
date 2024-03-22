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

#ifndef THERMIKSCHALE_H
#define THERMIKSCHALE_H

#include <iostream>

#include "thconf.h"
#include "solve.h"

class ThermikSchalen;

/**
 *
 * A ThermikSchalen consists of two ThermikSchale. One describes the inner 
 * part where air moves upwards, the other one describes the outer part 
 * where air moves downwards.
 * 
 * A ThermikSchale is stretched using a parameter t. t=0 is near the dead zone,
 * t=1 is the center of the thermal (inner part, air moving upwards) or the 
 * outmost point (outer part, air moving downwards).
 * 
 * A ThermikSchale consists of a straight line in the middle part (its length can 
 * be zero as well), an ellipsis at the upper end and an ellipsis at the lower end.
 * 
 * @author Jens W. Wulf
 */
class ThermikSchale
{
  public:   

   /**
    * y of dead zone, upper end.
    */
   flttype dzu_y;
   
   /**
    * y of dead zone, lower end.
    */
   flttype dzl_y;
   
   /**
    * y of straight line, upper end, t=0.
    */
   flttype iu_y;
   
   /**
    * y of straight line, lower end, t=0.
    */
   flttype il_y;
      
   /**
    * x of straight line, lower end, t=1.
    */
   flttype ol_x;
   
   /**
    * y of straight line, lower end, t=1.
    */
   flttype ol_y;
   
   /**
    * x of straight line, upper end, t=1.
    */
   flttype ou_x;
   
   /**
    * y of straight line, upper end, t=1.
    */
   flttype ou_y;

   void showpara();

   void init();
   
   /**
    * x, y:     der Punkt dessen Daten gesucht werden
    * dx, dy:   Richtung der Strömung in diesem Punkt
    * dxs, dys: Richtung der Senkrechten; die Länge entspricht einem bestimmten deltat.
    */
   void getDir(flttype x, flttype y, flttype& dx, flttype& dy, flttype& dxs, flttype& dys, flttype& t);

   void zeigSchaleOben(flttype t, std::ostream& out);
   void zeigSchaleUnten(flttype t, std::ostream& out);
   void zeigGerade(flttype t, std::ostream& out);
   
   void zeigVectSchaleOben(flttype t, std::ostream& out);
   void zeigVectSchaleUnten(flttype t, std::ostream& out);
   void zeigVectGerade(flttype t, std::ostream& out);

   flttype get_A_Ref(flttype t);
   
   ThermikSchalen* parent;
   
  private:
   
   flttype nenner;
   flttype A1;
   flttype A2;
   flttype A3;
   flttype A4;
   
   SolveFourthOrder solver;
};
#endif
