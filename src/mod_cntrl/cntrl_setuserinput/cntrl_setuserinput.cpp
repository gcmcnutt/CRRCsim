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
#include "cntrl_setuserinput.h"
#include "../../mod_misc/lib_conversions.h"

Cntrl_SetUserInput::Cntrl_SetUserInput(SimpleXMLTransfer* cfg)
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
  
  if (cfg->indexOfAttribute("file") < 0)
  {
    infile = 0;
  }
  else
  {
    infile = new SimpleXMLTransfer(cfg->getString("file"));
    idx    = 0;
    dTime  = 0;
  }
}

Cntrl_SetUserInput::~Cntrl_SetUserInput()
{
  if (infile)
    delete infile;
}

void Cntrl_SetUserInput::Interp(float*             vptr,
                                SimpleXMLTransfer* from,
                                SimpleXMLTransfer* to,
                                std::string        name,
                                double             fact)
{
  if (*vptr < 888 && from->indexOfAttribute(name) >= 0)
  {
    double v0 = from->getDouble(name);
    *vptr = v0 + (to->getDouble(name) - v0) * fact;
  }
}

void Cntrl_SetUserInput::Calc(double      dt, 
                              FDMBase*    fdm,
                              TSimInputs* pInputsFromUser,
                              TSimInputs* pInputsToFDM)
{
  if (infile)
  {
    dTime += dt;
  
    SimpleXMLTransfer* from;
    SimpleXMLTransfer* to;

    to = infile->getChildAt(idx+1);
    
    if (to->getDouble("time") < dTime)
    {
      if (infile->getChildCount() <= idx+2)
      {
        delete infile;
        infile = 0;
      }
      else                  
      {
        idx++;
        to = infile->getChildAt(idx+1);
      }
    }
     
    if (infile && to->getDouble("time") > dTime)
    {
      from = infile->getChildAt(idx);
      
      double time_from = from->getDouble("time");
      double time_to   = to  ->getDouble("time");

      double fact = (dTime-time_from)/(time_to-time_from);

      Interp(&flVal_aileron,  from, to, "aileron",  fact);
      Interp(&flVal_elevator, from, to, "elevator", fact);
      Interp(&flVal_rudder,   from, to, "rudder",   fact);
      Interp(&flVal_throttle, from, to, "throttle", fact);
      Interp(&flVal_flap,     from, to, "flap",     fact);
      Interp(&flVal_spoiler,  from, to, "spoiler",  fact);
      Interp(&flVal_retract,  from, to, "retract",  fact);
      Interp(&flVal_pitch,    from, to, "pitch",    fact);
      
      for (int n=0; n<NUM_AUX_INPUTS; n++)
        Interp(&(flVal_AUX[n]), from, to, "aux" + itoStr(n, ' ', 1), fact);
    }        
  }
  
  if (flVal_aileron < 888)
    pInputsFromUser->aileron  = flVal_aileron;
  if (flVal_elevator < 888)
    pInputsFromUser->elevator = flVal_elevator;
  if (flVal_rudder < 888)
    pInputsFromUser->rudder   = flVal_rudder;
  if (flVal_throttle < 888)
    pInputsFromUser->throttle = flVal_throttle;
  if (flVal_flap < 888)
    pInputsFromUser->flap     = flVal_flap;
  if (flVal_spoiler < 888)
    pInputsFromUser->spoiler  = flVal_spoiler;
  if (flVal_retract < 888)
    pInputsFromUser->retract  = flVal_retract;
  if (flVal_pitch < 888)
    pInputsFromUser->pitch    = flVal_pitch;
  
  for (int n=0; n<NUM_AUX_INPUTS; n++)
    if (flVal_AUX[n] < 888)
      pInputsFromUser->aux[n] = flVal_AUX[n];
}
