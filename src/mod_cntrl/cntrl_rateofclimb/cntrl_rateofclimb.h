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
#ifndef CNTRL_RATEOFCLIMB_H
# define CNTRL_RATEOFCLIMB_H

#include "../controller.h"

#include <fstream>

/**
 * This PI-controller alters elevator input to achieve a certain rate of climb.
 * Setpoint is taken from elevator stick input, so you should use InitInputs
 * before this controller (see below for an example). The plane has to be powered 
 * sufficiently to achieve a positive rate of climb. This controller doesn't set 
 * throttle; the pilot has to take care of it.
 * 
 * Please keep in mind that this is more something to play with, use as a starting 
 * point and show controller integration in CRRCSim than something to use...
 * 
 *   <InitInputs elevator="0" />
 *   <RateOfClimb kp="-0.02" ki="-0.02" scale="-50" />
 * 
 * @author Jens W. Wulf
 */
class Cntrl_RateOfClimb : public Controller
{
public:
  
  Cntrl_RateOfClimb(SimpleXMLTransfer* cfg);
  
  virtual void Reset();
  
  virtual void Calc(double      dt, 
                    FDMBase*    fdm,
                    TSimInputs* pInputsFromUser,
                    TSimInputs* pInputsToFDM);

  virtual ~Cntrl_RateOfClimb();
  
private:
  
  double kp, ki, kd, scale, dTSample, dTSampleDead, dAltLSB, dTCntrl, dIntLim;
  double dTSampleOutCnt, dTSampleCnt;
  double dInt;
  
  int nAltitude, nAltitudeFilt, nAltitudeOld;
    
  bool   fInit;
  
  double dROC; // rate of climb, m/s
  double dROCFilt; // rate of climb, m/s
  double dROCC;
  double dLastOut;
  
  double dTCntrlCnt;
  
  bool   fLog;
  
  std::ofstream outlog;
};

#endif
