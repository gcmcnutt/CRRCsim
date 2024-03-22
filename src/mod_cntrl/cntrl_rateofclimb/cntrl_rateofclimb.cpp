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
#include "cntrl_rateofclimb.h"

#include <iostream>

Cntrl_RateOfClimb::Cntrl_RateOfClimb(SimpleXMLTransfer* cfg)
{
  kp           = cfg->getDouble("kp");
  ki           = cfg->getDouble("ki");
  kd           = cfg->getDouble("kd");
  scale        = cfg->getDouble("scale");
  dTCntrl      = cfg->attributeAsDouble("TCntrl",       18.6E-3); // s
  dTSample     = cfg->attributeAsDouble("TSample",     168.0E-3); // s
  dTSampleDead = cfg->attributeAsDouble("TSampleDead",  84.0E-3); // s
  dAltLSB      = cfg->attributeAsDouble("AltLSB",     0.5308E-3); // m
  dIntLim      = cfg->attributeAsDouble("IntLim",           0.5);
  Reset();

  std::string logfilename = cfg->attribute("logfile", "");  
  if (logfilename.length())
  {
    fLog = true;
    outlog.open(logfilename.c_str());
    if (!outlog)
      std::cerr << "unable to open log file rateofclimb.dat\n";
    outlog << "#T " << dTCntrl << " s 2\n";
    outlog << "#N soll ist ist_filt hoehe integrator ausgang v\n";
    outlog << "#M 50 50 50 1000 5 5 100\n";
    outlog << "#U m/s m/s m/s m 1 1 m/s\n";
  }
  else
    fLog = false;
}

Cntrl_RateOfClimb::~Cntrl_RateOfClimb() 
{
  if (fLog)
    outlog.close();
}

void Cntrl_RateOfClimb::Reset()
{
  dROC           = 0;
  dROCFilt       = 0;
  dROCC          = 0;
  dInt           = 0;
  dTSampleOutCnt = 0;
  dTSampleCnt    = 0;
  fInit          = false;
  dLastOut       = 0;
  dTCntrlCnt     = 0;
}

void Cntrl_RateOfClimb::Calc(double      dt, 
                             FDMBase*    fdm,
                             TSimInputs* pInputsFromUser,
                             TSimInputs* pInputsToFDM)
{
  // --- model measurement device (dead time, filter) ---------------
  if (dTSampleOutCnt > 0)
  {
    dTSampleOutCnt -= dt;
    if (dTSampleOutCnt < 0)
    {
      if (fInit)
      {
        int diff  = (nAltitude - nAltitudeFilt) >> 3;
        int shift = 2;
        nAltitudeFilt += diff;
        
        double dROCFiltNew = ((diff >> shift) * dAltLSB / dTSample)*(1<<shift);
        dROCC = (dROCFiltNew - dROCFilt);
        dROCFilt     = dROCFiltNew;
        // calculate unfiltered ROC
        dROC         = (nAltitude - nAltitudeOld) * dAltLSB / dTSample;
        nAltitudeOld = nAltitude;
      }
      else
      {
        fInit         = true;
        nAltitudeFilt = nAltitude;
        nAltitudeOld  = nAltitude;
      }
    }
  }
  dTSampleCnt -= dt;
  if (dTSampleCnt < 0)
  {
    dTSampleCnt += dTSample;
    dTSampleOutCnt = dTSampleDead;
    nAltitude = (int)(fdm->getAlt() * 0.3048 / dAltLSB);
  }
      
  // --- calculate controller ---------------------------------------
  double res;
  dTCntrlCnt -= dt;
  if (dTCntrlCnt < 0)
  {
    dTCntrlCnt += dTCntrl;
    // calc setpoint
    double setp = pInputsFromUser->elevator*scale;
    
    // PI-controller
    const double max = 0.5;
    res  = (setp-dROCFilt)*kp + dROCC*kd + dInt;
    if (res > max)
      res = max;
    else if (res < -max)
      res = -max;
    else
    {
      dInt += (setp-dROC)*ki*dTCntrl;
      if (dInt > dIntLim)
        dInt = dIntLim;
      else if (dInt < -dIntLim)
        dInt = -dIntLim;
    }
    
    dLastOut = res;
    if (fLog)
    {
      outlog << setp << " "
        << dROC << " " << dROCFilt << " " << (fdm->getAlt() * 0.3048) << " "
        << dInt << " " << (pInputsToFDM->elevator+res) << " "
        << fdm->getVRelAirmass()*0.3048 << "\n";
    }
  }
  else
    res = dLastOut;
  
  // --- apply output of controller ---------------------------------
  pInputsToFDM->elevator += res;
  Limit(pInputsToFDM->elevator);
}
