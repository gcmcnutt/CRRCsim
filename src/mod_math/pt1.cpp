/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2008 - Jens Wilhelm Wulf (original author)
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
#include "pt1.h"

#include <math.h>

namespace CRRCMath
{
  void PT1::init(double ival, double zeitkonstante)
  {
    init(ival);
    SetTau(zeitkonstante);
  }

  void PT1::init(double ival)
  {
    val      = ival;
    d_dt_Alt = -1;
  }

  void PT1::SetTau(double zeitkonstante)
  {
    dZeitkonstante = zeitkonstante;
  }

  void PT1::step(double dt, double nval)
  {
    if (d_dt_Alt != dt)
    {
      if (dZeitkonstante == 0)
        d_Mul = 1;
      else
        d_Mul = 1-exp(-dt/dZeitkonstante);
      d_dt_Alt = dt;
    }

    val += (nval-val)*d_Mul;
  }
};
