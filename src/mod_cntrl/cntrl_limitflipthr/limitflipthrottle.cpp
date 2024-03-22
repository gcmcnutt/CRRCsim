/*
 * Crrcsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2009 - Jens Wilhelm Wulf (original author)
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
#include "limitflipthrottle.h"

Cntrl_LimitFlipThrottle::Cntrl_LimitFlipThrottle(SimpleXMLTransfer* cfg)
{
  max90  = cfg->getDouble("max90");
  max180 = cfg->getDouble("max180");
}

Cntrl_LimitFlipThrottle::~Cntrl_LimitFlipThrottle() 
{
}

void Cntrl_LimitFlipThrottle::Calc(double      dt,
                                   FDMBase*    fdm,
                                   TSimInputs* pInputsFromUser,
                                   TSimInputs* pInputsToFDM)
{
  CRRCMath::Vector3 diff = CRRCMath::Vector3(0, 0, 1) - fdm->WorldToBody(CRRCMath::Vector3(0, 0, 1));
  double qdiff = diff.r[0]*diff.r[0] + diff.r[1]*diff.r[1] + diff.r[2]*diff.r[2];  
  double max = 1; // assume no limit
  
  if (qdiff > 2) // at least 90°
  {
    max = max90 + (qdiff-2)/2*(max180-max90);
  }
  else if (qdiff > 1) // more than 60°, but less than 90° 
  {
    max = 1 + (qdiff-1)*(max90-1);
  }

  if (pInputsToFDM->throttle > max)
    pInputsToFDM->throttle = max;
}
