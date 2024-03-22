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
#include "cntrl_mcopter01.h"
#include "../../mod_fdm/xmlmodelfile.h"
#include "../../mod_misc/lib_conversions.h"

#include <iostream>

Cntrl_MCopter01::Cntrl_MCopter01(SimpleXMLTransfer* cfg)
{
  controllerA[0].ReadData(cfg->getChild("roll"));
  controllerA[1].ReadData(cfg->getChild("pitch"));  
  cntrlOmega[2].ReadData( cfg->getChild("yaw"));
  if (cfg->indexOfChild("Omega") >= 0)
  {
    SimpleXMLTransfer* cfg_omega = cfg->getChild("Omega");
    cntrlOmega[0].ReadData(cfg_omega->getChild("roll"));
    cntrlOmega[1].ReadData(cfg_omega->getChild("pitch"));
    
    scale_a   = cfg_omega->getDouble("scale_roll_pitch.a",   1);
    scale_b   = cfg_omega->getDouble("scale_roll_pitch.b",   0);
    scale_exp = cfg_omega->getDouble("scale_roll_pitch.exp", 0) - 1;
    
    smAttitudeRate = XMLModelFile::GetSteering(strU(cfg_omega->attribute("switch_channel")));
  }    
  else
    smAttitudeRate = TSimInputs::smNOTHING;
  Reset();

  std::string logfilename = cfg->attribute("logfile", "");  
  if (logfilename.length())
  {
    nLog = 2;
    outlog.open(logfilename.c_str());
    if (!outlog)
      std::cerr << "unable to open log file " << logfilename << "\n";
    outlog << "#T todo s 2\n";
    outlog << "#N p     q     r     phi theta phi_ theta_ r_\n";
    outlog << "#M 99    99    99    9   9     9    9      99\n";
    outlog << "#U rad/s rad/s rad/s rad rad   rad  rad    rad/s\n";
  }
  else
    nLog = 0;

  nMode = 0;
}

Cntrl_MCopter01::~Cntrl_MCopter01() 
{
  if (nLog)
    outlog.close();
}

void Cntrl_MCopter01::Reset()
{
  for (int n=0; n<3; n++)
  {
    cntrlOmega[n].integrator = 0;
    cntrlOmega[n].val_old    = 0;
    cntrlOmega[n].filter.init(0);
  }
}

void Cntrl_MCopter01::Calc(double      dt, 
                           FDMBase*    fdm,
                           TSimInputs* pInputsFromUser,
                           TSimInputs* pInputsToFDM)
{
  CRRCMath::Vector3 omega = fdm->getPQR();

  pInputsToFDM->rudder = cntrlOmega[2].Step(dt, pInputsFromUser->rudder, omega.r[2]);
  
  if (smAttitudeRate == TSimInputs::smNOTHING || pInputsFromUser->GetInput(smAttitudeRate) > 0.25)
  {
    pInputsToFDM->aileron  = controllerA[0].Step(dt, pInputsFromUser->aileron,  fdm->getPhi(),   omega.r[0]);
    pInputsToFDM->elevator = controllerA[1].Step(dt, pInputsFromUser->elevator, fdm->getTheta(), omega.r[1]);
    
    cntrlOmega[0].Step(dt, Scale(pInputsFromUser->aileron),  omega.r[0]);
    cntrlOmega[1].Step(dt, Scale(pInputsFromUser->elevator), omega.r[1]);
    cntrlOmega[0].integrator = 0;
    cntrlOmega[1].integrator = 0;
    
    if (nMode != 1)
    {
      fdm->GetEnv()->AddLogMsg("cntrl_mcopter01: attitude controlled");      
      nMode = 1;
    }
  }
  else
  {
    pInputsToFDM->aileron  = cntrlOmega[0].Step(dt, Scale(pInputsFromUser->aileron),  omega.r[0]);
    pInputsToFDM->elevator = cntrlOmega[1].Step(dt, Scale(pInputsFromUser->elevator), omega.r[1]);
    
    if (nMode != 2)
    {
      fdm->GetEnv()->AddLogMsg("cntrl_mcopter01: rate controlled");      
      nMode = 2;
    }
  }
  
  if (pInputsFromUser->KeyPressed('l'))
  {
    switch (nLog)
    {
      case 1:
        fdm->GetEnv()->AddLogMsg("cntrl_mcopter01: log started");
        nLog = 2;
        break;
      case 2:
        fdm->GetEnv()->AddLogMsg("cntrl_mcopter01: log stopped");
        nLog = 1;
        break;
      default:
        fdm->GetEnv()->AddLogMsg("cntrl_mcopter01: no log file set");
        break;
    }
  }
  
  if (nLog == 2)
  {
    outlog << omega.r[0] << " " << omega.r[1] << " " << omega.r[2] << " "
      << fdm->getPhi() << " " << fdm->getTheta() << " "
      << controllerA[0].GetSetpoint(pInputsFromUser->aileron) << " " 
      << controllerA[1].GetSetpoint(pInputsFromUser->elevator) << " " 
      << cntrlOmega[2].GetSetpoint(pInputsFromUser->rudder) << "\n";
  }
}

double Cntrl_MCopter01::Scale(double in)
{
  if (scale_exp > 1)
    return( scale_a * in + scale_b * in * pow(fabs(in), scale_exp) );
  else
    return(in);
}
