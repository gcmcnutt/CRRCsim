/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2006, 2008, 2009 - Jens Wilhelm Wulf (original author)
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
#include "shaft.h"

#include <iostream>
#include "engine_dcm.h"
#include "propeller.h"
#include "simplethrust.h"

Power::Shaft::Shaft()
{
  gear.push_back(new Propeller());
  gear.push_back(new Engine_DCM());
}

Power::Shaft::Shaft(SimpleXMLTransfer* xml)
{    
  for (int n=0; n<xml->getChildCount(); n++)
  {
    SimpleXMLTransfer* it = xml->getChildAt(n);
    Gearing*           s  = (Gearing*)0;
    if (it->getName().compare("engine") == 0)
      s = new Engine_DCM();
    else if (it->getName().compare("propeller") == 0)
      s = new Propeller();
    else if (it->getName().compare("simplethrust") == 0)
      s = new SimpleThrust();
    if (s != (Gearing*)0)
      gear.push_back(s);
  }
}

void Power::Shaft::ReloadParams(SimpleXMLTransfer* xml)
{
  int      nChildCnt = 0;
  double   J_ges;
  
  fBrake = (xml->attributeAsInt("brake", 1) != 0);
  J_ges = xml->attributeAsDouble("J", 0);
  
  std::cout << "Shaft      : J=" << J_ges << " kg m^2\n";
      
  for (int n=0; n<xml->getChildCount(); n++)
  {
    SimpleXMLTransfer* it = xml->getChildAt(n);
    if (it->getName().compare("engine")       == 0 ||
        it->getName().compare("propeller")    == 0 ||
        it->getName().compare("simplethrust") == 0
        )
    {
      gear[nChildCnt]->ReloadParams(it);
      J_ges += gear[nChildCnt++]->getJ();
    }
  }
  
  J_inv = 1/J_ges;
}

void Power::Shaft::ReloadParams_automagic(SimpleXMLTransfer* xml)
{
  fBrake = (xml->getChild("battery.shaft")->attributeAsInt("brake", 1) != 0);
  
  for (unsigned int i = 0; i < gear.size(); i++)
    gear[i]->ReloadParams_automagic(xml);
    
  J_inv = 1/(gear[0]->getJ() + gear[1]->getJ());
}

Power::Shaft::~Shaft()
{
  // deallocate gears
  for (unsigned int i = 0; i < gear.size(); i++)
  {
    delete gear[i];
    gear[i] = NULL;
  }
}

void Power::Shaft::step(PowerValuesStep* values)
{
  values->omega        = omega.val;
  values->moment_shaft = 0;
  
  int size = gear.size();    
  for (int n=0; n<size; n++)
  {
    gear[n]->step(values);
  }
  
  if ( (values->inputs->throttle < 0.05 || values->U < 0.01) && fBrake)
  {
    omega.init(0, 0);
  }
  else
  {
    omega.step(values->dt, values->moment_shaft*J_inv);
  }
}

void Power::Shaft::InitStates(CRRCMath::Vector3 vInitialVelocity)
{
  // There may be more than one propeller connected to me, but
  // I just set my initial omega according to what one of the props 
  // tells me. However, if there is more than one prop, most surely all of them
  // will have the same gearbox ratio and pitch so this is not a problem
  // at all.
  omega.init(0, 0);
  double dOmega = 0;
  for (unsigned int n=0; n<gear.size(); n++)
    gear[n]->InitStates(vInitialVelocity, dOmega);
  
  omega.init(dOmega, 0);  
}
