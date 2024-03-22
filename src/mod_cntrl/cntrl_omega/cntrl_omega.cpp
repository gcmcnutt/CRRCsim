/*
 * Crrcsim - the Charles River Radio Control Club Flight Simulator Project
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
#include "cntrl_omega.h"

#include <iostream>

Cntrl_Omega::Cntrl_Omega(SimpleXMLTransfer* cfg)
{
  scale_a   = cfg->getDouble("scale_roll_pitch.a",   1);
  scale_b   = cfg->getDouble("scale_roll_pitch.b",   0);
  scale_exp = cfg->getDouble("scale_roll_pitch.exp", 0) - 1;
  
  controllers[0].ReadData(cfg->getChild("roll"));
  controllers[1].ReadData(cfg->getChild("pitch"));
  controllers[2].ReadData(cfg->getChild("yaw"));  
  Reset();

  std::string logfilename = cfg->attribute("logfile", "");  
  if (logfilename.length())
  {
    fLog = true;
    outlog.open(logfilename.c_str());
    if (!outlog)
      std::cerr << "unable to open log file omega.dat\n";
    outlog << "#T todo s 2\n";
    outlog << "#N soll ist ist_filt hoehe integrator ausgang v\n";
    outlog << "#M 50 50 50 1000 5 5 100\n";
    outlog << "#U m/s m/s m/s m 1 1 m/s\n";
  }
  else
    fLog = false;
}

Cntrl_Omega::~Cntrl_Omega() 
{
  if (fLog)
    outlog.close();
}

void Cntrl_Omega::Reset()
{
  for (int n=0; n<3; n++)
  {
    controllers[n].integrator = 0;
    controllers[n].val_old    = 0;
    controllers[n].filter.init(0);
  }
}

void Cntrl_Omega::Calc(double      dt,
                       FDMBase*    fdm,
                       TSimInputs* pInputsFromUser,
                       TSimInputs* pInputsToFDM)
{
  CRRCMath::Vector3 omega = fdm->getPQR();
  
  pInputsToFDM->aileron  = controllers[0].Step(dt, Scale(pInputsFromUser->aileron),  omega.r[0]);
  pInputsToFDM->elevator = controllers[1].Step(dt, Scale(pInputsFromUser->elevator), omega.r[1]);
  pInputsToFDM->rudder   = controllers[2].Step(dt, pInputsFromUser->rudder,          omega.r[2]);
}

double Cntrl_Omega::Scale(double in)
{
  if (scale_exp > 1)
    return( scale_a * in + scale_b * in * pow(fabs(in), scale_exp) );
  else
    return(in);
}
