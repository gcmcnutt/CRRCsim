/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2008 - Jens Wilhelm Wulf (original author)
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
#include "controller.h"

#include <iostream>

#include "cntrl_initinputs/cntrl_initinputs.h"
#include "cntrl_rateofclimb/cntrl_rateofclimb.h"
#include "cntrl_phugoid/cntrl_phugoid.h"
#include "cntrl_setuserinput/cntrl_setuserinput.h"
#include "cntrl_omega/cntrl_omega.h"
#include "cntrl_mcopter01/cntrl_mcopter01.h"
#include "cntrl_scalethrottle/scalethrottle.h"
#include "cntrl_limitflipthr/limitflipthrottle.h"


void Controller::LoadList(SimpleXMLTransfer*       cfg, 
                          std::vector<Controller*> &controllers)
{
  try
  {
    SimpleXMLTransfer* cntrldescr;
    Controller*        cntrl;
    std::string name;
    for (int n=0; n<cfg->getChildCount(); n++)
    {
      cntrldescr = cfg->getChildAt(n);
      name       = cntrldescr->getName();
      cntrl      = 0;
      /*
       * MNav is not converted to this structure yet but should be...
      if (name.compare("MNAV") == 0)
        cntrl = new Cntrl_MNAV(cntrldescr);
      else */
      if (name.compare("InitInputs") == 0)
        cntrl = new Cntrl_InitInputs(cntrldescr);
      else if (name.compare("SetUserInput") == 0)
        cntrl = new Cntrl_SetUserInput(cntrldescr);
      else if (name.compare("RateOfClimb") == 0)
        cntrl = new Cntrl_RateOfClimb(cntrldescr);
      else if (name.compare("Phugoid") == 0)
        cntrl = new Cntrl_Phugoid(cntrldescr);
      else if (name.compare("Omega") == 0)
        cntrl = new Cntrl_Omega(cntrldescr);
      else if (name.compare("MCopter01") == 0)
        cntrl = new Cntrl_MCopter01(cntrldescr);
      else if (name.compare("ScaleThrottle") == 0)
        cntrl = new Cntrl_ScaleThrottle(cntrldescr);
      else if (name.compare("LimitFlipThrottle") == 0)
        cntrl = new Cntrl_LimitFlipThrottle(cntrldescr);
             
      if (cntrl)
        controllers.push_back(cntrl);
    }
  }
  catch (XMLException e)
  {
    std::cerr << "XMLException when initializing controllers: " << e.what() << "\n";
  }  
}

int Controller::Limit(float &flVal)
{
  if (flVal > 0.5)
  {
    flVal = 0.5;
    return(1);
  }
  else if (flVal < -0.5)
  {
    flVal = -0.5;
    return(-1);
  }
  else
    return(0);
}
