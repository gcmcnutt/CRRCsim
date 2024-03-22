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
#include "scalethrottle.h"

Cntrl_ScaleThrottle::Cntrl_ScaleThrottle(SimpleXMLTransfer* cfg)
{
  off = cfg->getDouble("off");
  min = cfg->getDouble("min");
  mul = cfg->getDouble("mul");
}

Cntrl_ScaleThrottle::~Cntrl_ScaleThrottle() 
{
}

void Cntrl_ScaleThrottle::Calc(double      dt,
                               FDMBase*    fdm,
                               TSimInputs* pInputsFromUser,
                               TSimInputs* pInputsToFDM)
{
  double bla = pInputsFromUser->throttle - off;
  
  if (bla < 0)
    pInputsToFDM->throttle = 0;
  else
    pInputsToFDM->throttle = min + bla * mul;
}
