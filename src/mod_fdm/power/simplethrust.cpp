/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2008 - Jens Wilhelm Wulf (original author)
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
#include "simplethrust.h"

#include <iostream>


Power::SimpleThrust::SimpleThrust() : Gearing()
{
  J = 0;
}

void Power::SimpleThrust::ReloadParams(SimpleXMLTransfer* xml)
{
  Gearing::ReloadParams(xml);
  
  k_F = xml->getDouble("k_F");
  k_M = xml->getDouble("k_M");
  
  std::cout << "SimpleThrust: k_F=" << k_F << ", k_M=" << k_M << "\n";
}

void Power::SimpleThrust::step(PowerValuesStep* values)
{
  double  omega = i*values->omega;
  double  n     = omega/(2*M_PI);

  values->force->r[0]  += omega*k_F;
  values->moment_shaft -= omega*k_M*i;
  values->dPropFreq     = n;
}
