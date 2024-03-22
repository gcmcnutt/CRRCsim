/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2008, 2009 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2006 - Jan Reucker
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
#include "power.h"

#include "values_step.h"

#define POWER_DT 2.7E-3

Power::Power::Power()
{
  dPropFreq = 0;
}

Power::Power::Power(SimpleXMLTransfer* xml, int nVerbosity)
{
  SimpleXMLTransfer* power = xml->getChild("power");
  SimpleXMLTransfer* bat;

  dVoltageAvg = 1;
  
  // --- create system ---------------------------------
  for (int n=0; n<power->getChildCount(); n++)
  {
    bat = power->getChildAt(n);
    if (bat->getName().compare("battery") == 0)
      batteries.push_back(new Battery(bat));
    else if (bat->getName().compare("automagic") == 0)
      batteries.push_back(new Battery());
  }

  // --- read parameters -------------------------------
  ReloadParams(xml, nVerbosity);
  
  // --- init states -----------------------------------
  InitStates(CRRCMath::Vector3());
}

void Power::Power::ReloadParams(SimpleXMLTransfer* xml, int nVerbosity)
{
  SimpleXMLTransfer* power = xml->getChild("power");
  SimpleXMLTransfer* bat;
  int nBatCnt = 0;

  for (int n=0; n<power->getChildCount(); n++)
  {
    bat = power->getChildAt(n);
    if (bat->getName().compare("battery") == 0)
    {
      batteries[nBatCnt++]->ReloadParams(bat);
    }
    else if (bat->getName().compare("automagic") == 0)
    {
      SimpleXMLTransfer* realcfg;
      
      if (nVerbosity > 4)
      {
        std::cout << "  --- automatic power configuration: input -------------------------\n";
        bat->print(std::cout, 2);
      }

      batteries[nBatCnt++]->ReloadParams_automagic(bat);

      {
        // remove automagic entries
        SimpleXMLTransfer* pa;
        SimpleXMLTransfer* gr;
        int                idx;

        pa  = bat->getChild("battery");
        idx = pa->indexOfChild("automagic");
        gr  = pa->getChildAt(idx);
        pa->removeChildAt(idx);
        delete gr;

        pa  = bat->getChild("battery.shaft.engine");
        idx = pa->indexOfChild("automagic");
        gr  = pa->getChildAt(idx);
        pa->removeChildAt(idx);
        delete gr;

        pa  = bat->getChild("battery.shaft.propeller");
        idx = pa->indexOfChild("automagic");
        gr  = pa->getChildAt(idx);
        pa->removeChildAt(idx);
        delete gr;
        
        // replace automagic by real config
        realcfg = new SimpleXMLTransfer(bat->getChild("battery"));        
        power->replaceChild(bat, realcfg);
        delete bat;        
      }

      if (nVerbosity > 3)
      {
        std::cout << "  --- automatic power configuration: output ------------------------\n";
        realcfg->print(std::cout, 2);
        std::cout << "  --- automatic power configuration: end ---------------------------\n";
      }
    }
  }
}

Power::Power::~Power()
{
  // deallocate all batteries
  for (unsigned int i = 0; i < batteries.size(); i++)
  {
    delete batteries[i];
    batteries[i] = NULL;
  }
}


void Power::Power::step(double             dt,
                        TSimInputs*        inputs,
                        CRRCMath::Vector3  VRelAir,
                        CRRCMath::Vector3* force,
                        CRRCMath::Vector3* moment)
{
  const int       mul = 2;
  unsigned int    size = batteries.size();
  PowerValuesStep values;
  CRRCMath::Vector3         f = CRRCMath::Vector3();
  CRRCMath::Vector3         m = CRRCMath::Vector3();

  values.force          = &f;
  values.moment         = &m;
  values.inputs         = inputs;
  values.VRelAir        = VRelAir;
  values.dPropFreq      = 0; // in case of an empty system
  values.dBatCapLeftMin = 1;
  
  dVoltageAvg = 0;
  
  // Looks like this needs to be simulated at a higher frequency.
  values.dt     = dt/mul;

  for (unsigned int m=0; m<mul-1; m++)
  {
    for (unsigned int n=0; n<size; n++)
    {
      batteries[n]->step(&values);
    }
  }

  values.force  = force;
  values.moment = moment;
  for (unsigned int n=0; n<size; n++)
  {
    batteries[n]->step(&values);
    dVoltageAvg += values.U;
  }

  dPropFreq      = values.dPropFreq;
  dBatCapLeftMin = values.dBatCapLeftMin;
  dVoltageAvg    = dVoltageAvg / size;
}

void Power::Power::InitStates(CRRCMath::Vector3 vInitialVelocity)
{
  unsigned int size = batteries.size();
  
  for (unsigned int n=0; n<size; n++)
    batteries[n]->InitStates(vInitialVelocity);
  
  dPropFreq = 0;
}

void Power::Power::Sim_UntilStable(TSimInputs*        inputs,
                                   CRRCMath::Vector3  VRelAir,
                                   double             lim,
                                   CRRCMath::Vector3* force,
                                   CRRCMath::Vector3* moment)
{
  bool fRun = true;
  CRRCMath::Vector3 f_o = CRRCMath::Vector3();
  CRRCMath::Vector3 m_o = CRRCMath::Vector3();
  
  while (fRun)
  {
    *force  = CRRCMath::Vector3();
    *moment = CRRCMath::Vector3();
    
    step(POWER_DT, inputs, VRelAir, force, moment);
    
    fRun = false;
    
    if (moment->length())
      if (fabs((*moment - m_o).length()) / moment->length() > lim)
        fRun = true;
    
    if (force->length())
      if (fabs((*force - f_o).length()) / force->length() > lim)
        fRun = true;
    
    f_o = *force;
    m_o = *moment;
  }
}

float Power::Power::Sim_GetThrottle(CRRCMath::Vector3  VRelAir, float force, float pitch, CRRCMath::Vector3& torque)
{
  float stepsize = 0.25;
  TSimInputs inp;
  CRRCMath::Vector3 F;
  
  inp.pitch    = pitch;
  inp.throttle = 0.5;
  
  // use binary search method
  while (stepsize > 0.0001)
  {
    F = CRRCMath::Vector3();
    torque = CRRCMath::Vector3();
    Sim_UntilStable(&inp, VRelAir, 0.00001, &F, &torque);
    
    if (F.length() > force)
      inp.throttle -= stepsize;
    else
      inp.throttle += stepsize;
    stepsize *= 0.5;    
  }
  return(inp.throttle);
}
    
float Power::Power::Sim_GetPitch(CRRCMath::Vector3  VRelAir, float force, float throttle, CRRCMath::Vector3& torque)
{
  float stepsize = 1;
  TSimInputs inp;
  CRRCMath::Vector3 F;
  
  inp.pitch    = 2;
  inp.throttle = throttle;
  
  // use binary search method
  while (stepsize > 0.0001)
  {
    F = CRRCMath::Vector3();
    torque = CRRCMath::Vector3();
    Sim_UntilStable(&inp, VRelAir, 0.00001, &F, &torque);
    
    if (F.length() > force)
      inp.pitch -= stepsize;
    else
      inp.pitch += stepsize;
    stepsize *= 0.5;    
  }
  
  return(inp.pitch);
}
