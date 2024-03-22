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
#ifndef CNTRL_INITINPUTS_H
# define CNTRL_INITINPUTS_H

#include "../controller.h"

/**
 * This is not really a controller, but provides a method to initialize 
 * control inputs. Every single input can be left as is or be set to a certain value.
 * 
 * XML description example:
 *  <InitInputs aileron="-0.2" aux2="0" rudder="0" />
 * Elements mentioned in the description will be set to the value given, others are 
 * left untouched.
 * 
 * @author Jens W. Wulf
 */
class Cntrl_InitInputs : public Controller
{
public:
  
  Cntrl_InitInputs(SimpleXMLTransfer* cfg);
    
  /**
   * Sets pInputsToFDM according to configuration.
   */
  virtual void Calc(double      dt, 
                    FDMBase*    fdm,
                    TSimInputs* pInputsFromUser,
                    TSimInputs* pInputsToFDM);

  virtual ~Cntrl_InitInputs() {};
  
private:
  enum { NUM_AUX_INPUTS = 4};  
  float flVal_AUX[NUM_AUX_INPUTS];
  
  float flVal_aileron; 
  float flVal_elevator;  
  float flVal_rudder;  
  float flVal_throttle;  
  float flVal_flap;  
  float flVal_spoiler;  
  float flVal_retract;  
  float flVal_pitch;  
};

#endif
