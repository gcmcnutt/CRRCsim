/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2006, 2008 - Jens Wilhelm Wulf (original author)
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
#ifndef LINEARREG_H
# define LINEARREG_H

#include <vector>

/**
 * Linear regression
 *
 *  y = a + b * x
 * 
 * @author Jens W. Wulf
 */
class T_LinearReg
{
  public:
   void   init();
   double get_a();
   double get_b();   
   void   add(double ix, double iy);      
   void   calc();
   
  private:
   std::vector<double> x;
   std::vector<double> y;
   double b;
   double a;
};

#endif
