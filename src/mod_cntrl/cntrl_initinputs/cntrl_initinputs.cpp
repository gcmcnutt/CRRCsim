/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2008 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2008 - Jan Reucker
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
#include "cntrl_initinputs.h"
#include "../../mod_misc/lib_conversions.h"

Cntrl_InitInputs::Cntrl_InitInputs(SimpleXMLTransfer* cfg)
{
  flVal_aileron  = cfg->attributeAsDouble("aileron",  999);
  flVal_elevator = cfg->attributeAsDouble("elevator", 999);
  flVal_rudder   = cfg->attributeAsDouble("rudder",   999);
  flVal_throttle = cfg->attributeAsDouble("throttle", 999);
  flVal_flap     = cfg->attributeAsDouble("flap",     999);
  flVal_spoiler  = cfg->attributeAsDouble("spoiler",  999);
  flVal_retract  = cfg->attributeAsDouble("retract",  999);
  flVal_pitch    = cfg->attributeAsDouble("pitch",    999);
  
  for (int n=0; n<NUM_AUX_INPUTS; n++)
    flVal_AUX[n] = cfg->attributeAsDouble("aux" + itoStr(n, ' ', 1), 999);
}

void Cntrl_InitInputs::Calc(double      dt, 
                            FDMBase*    fdm,
                            TSimInputs* pInputsFromUser,
                            TSimInputs* pInputsToFDM)
{
  if (flVal_aileron < 888)
    pInputsToFDM->aileron  = flVal_aileron;
  if (flVal_elevator < 888)
    pInputsToFDM->elevator = flVal_elevator;
  if (flVal_rudder < 888)
    pInputsToFDM->rudder   = flVal_rudder;
  if (flVal_throttle < 888)
    pInputsToFDM->throttle = flVal_throttle;
  if (flVal_flap < 888)
    pInputsToFDM->flap     = flVal_flap;
  if (flVal_spoiler < 888)
    pInputsToFDM->spoiler  = flVal_spoiler;
  if (flVal_retract < 888)
    pInputsToFDM->retract  = flVal_retract;
  if (flVal_pitch < 888)
    pInputsToFDM->pitch    = flVal_pitch;
  
  for (int n=0; n<NUM_AUX_INPUTS; n++)
    if (flVal_AUX[n] < 888)
      pInputsToFDM->aux[n] = flVal_AUX[n];
}
