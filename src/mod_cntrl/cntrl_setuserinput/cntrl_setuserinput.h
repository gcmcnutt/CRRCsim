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
#ifndef CNTRL_SETUSERINPUT_H
# define CNTRL_SETUSERINPUT_H

#include "../controller.h"

/**
 * This is not really a controller, but provides a method to set
 * user control inputs to values read from a file.
 * It can be used to automatically test controllers, and has no use for an interactive
 * simulation. This is why it is not located in 'mod_inputdev', but in 'mod_cntrl'.
 * 
 * XML description example:
 *  <SetUserInput aileron="-0.2" aux2="0" rudder="0" file="foo.xml" />
 * 
 * It reads time dependent values from a file, interpolates between them and returns those 
 * values. This feature is not well documented yet it is quite picky about input data, too.
 * 
 * @author Jens W. Wulf
 */
class Cntrl_SetUserInput : public Controller
{
public:
  
  Cntrl_SetUserInput(SimpleXMLTransfer* cfg);
    
  /**
   * Sets pInputsFromUser according to configuration.
   */
  virtual void Calc(double      dt, 
                    FDMBase*    fdm,
                    TSimInputs* pInputsFromUser,
                    TSimInputs* pInputsToFDM);

  virtual ~Cntrl_SetUserInput();
  
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
  
  double dTime;  
  SimpleXMLTransfer* infile;
  int idx;
  
  void Interp(float*             vptr,
              SimpleXMLTransfer* from,
              SimpleXMLTransfer* to,
              std::string        name,
              double             fact);
  
};

#endif
