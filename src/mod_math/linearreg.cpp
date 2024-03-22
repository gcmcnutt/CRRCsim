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
#include "linearreg.h"


void T_LinearReg::init()
{
  x.clear();
  y.clear();
}

void T_LinearReg::calc()
{
  double s1=0, s2=0, xquer=0, yquer=0;

  for (unsigned int i=0; i<x.size(); i++)
    {
      s1 += y[i]*x[i];
      s2 += x[i]*x[i];
      xquer += x[i];
      yquer += y[i];
    }
  xquer /= x.size();
  yquer /= x.size();

  b = (s1 - x.size()*xquer*yquer) / (s2 - x.size()*xquer*xquer);
  a = yquer - b * xquer;
}

void T_LinearReg::add(double ix, double iy)
{
  x.push_back(ix);
  y.push_back(iy);
}

double T_LinearReg::get_b()
{
  return(b);
}

double T_LinearReg::get_a()
{
  return(a);
}
