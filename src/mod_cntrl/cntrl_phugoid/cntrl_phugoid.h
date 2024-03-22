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
#ifndef CNTRL_PHUGOID_H
# define CNTRL_PHUGOID_H

#include "../controller.h"

#include <fstream>

/**
 * This controller takes care of phugoid mode.
 * 
 * Please keep in mind that this is more something to play with, use as a starting 
 * point and show controller integration in CRRCSim than something to use...
 * 
 *   <Phugoid kd="-0.02" />
 * 
 * @author Jens W. Wulf
 */
class Cntrl_Phugoid : public Controller
{
public:
  
  Cntrl_Phugoid(SimpleXMLTransfer* cfg);
  
  virtual void Reset();
  
  virtual void Calc(double      dt, 
                    FDMBase*    fdm,
                    TSimInputs* pInputsFromUser,
                    TSimInputs* pInputsToFDM);

  virtual ~Cntrl_Phugoid();
  
private:
  
  double kd, dTCntrl;
  double dTCntrlCnt;
  
  double dVOld; // velocity in m/s
    
  bool   fInit;
  double dLastOut;
  
  bool   fLog;
  
  std::ofstream outlog;
};

#endif
