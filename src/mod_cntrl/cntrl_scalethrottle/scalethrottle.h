/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
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
#ifndef CNTRL_SCALETHROTTLE_H
# define CNTRL_SCALETHROTTLE_H

#include "../controller.h"


/**
 * This is not a controller, but scales the throttle input.
 * 
 * @author Jens W. Wulf
 */
class Cntrl_ScaleThrottle : public Controller
{
public:
  
  Cntrl_ScaleThrottle(SimpleXMLTransfer* cfg);
  
  virtual void Reset() {};
  
  virtual void Calc(double      dt, 
                    FDMBase*    fdm,
                    TSimInputs* pInputsFromUser,
                    TSimInputs* pInputsToFDM);

  virtual ~Cntrl_ScaleThrottle();
      
private:

  double off;
  double min;
  double mul;

};

#endif
