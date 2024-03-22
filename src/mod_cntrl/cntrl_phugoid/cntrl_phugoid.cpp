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
#include "cntrl_phugoid.h"

Cntrl_Phugoid::Cntrl_Phugoid(SimpleXMLTransfer* cfg)
{
  kd           = cfg->getDouble("kd");
  dTCntrl      = cfg->attributeAsDouble("TCntrl", 18.6E-3); // s
  Reset();
  
  
  std::string logfilename = cfg->attribute("logfile", "");  
  if (logfilename.length())
  {
    fLog = true;
    outlog.open(logfilename.c_str());
    if (!outlog)
      std::cerr << "unable to open log file rateofclimb.dat\n";
    outlog << "#T " << dTCntrl << " s 2\n";
    outlog << "#N ausgang ausgang_ges v\n";
    outlog << "#M 5 5 100\n";
    outlog << "#U 1 1 m/s\n";
  }
  else
    fLog = false;
}

Cntrl_Phugoid::~Cntrl_Phugoid() 
{
  if (fLog)
    outlog.close();
}

void Cntrl_Phugoid::Reset()
{
  fInit          = false;
  dTCntrlCnt     = 0;
}

void Cntrl_Phugoid::Calc(double      dt, 
                         FDMBase*    fdm,
                         TSimInputs* pInputsFromUser,
                         TSimInputs* pInputsToFDM)
{
  // --- calculate controller ---------------------------------------
  float res;
  dTCntrlCnt -= dt;
  if (dTCntrlCnt < 0)
  {
    dTCntrlCnt += dTCntrl;
   
    if (fInit)
    {
      double dV     = fdm->getVRelAirmass()*0.3048;
      double dVDiff = dV - dVOld;
      dVOld = dV;
      
      res = kd * dVDiff;
      Limit(res);
    }
    else
    {
      fInit = true;
      dVOld = fdm->getVRelAirmass()*0.3048;
      res   = 0;
    }            
    dLastOut = res;
    
    if (fLog)
    {
      outlog << res << " " << (pInputsToFDM->elevator+res) << " "
        << fdm->getVRelAirmass()*0.3048 << "\n";      
    }
  }
  else
    res = dLastOut;
  
  // --- apply output of controller ---------------------------------
  pInputsToFDM->elevator += res;
  Limit(pInputsToFDM->elevator);
}
