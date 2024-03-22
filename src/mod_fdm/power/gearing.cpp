/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2006, 2008 - Jens Wilhelm Wulf (original author)
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
#include "gearing.h"

#include <iostream>

Power::Gearing::Gearing()
{
  // set default values
  i   = 1;
  J_G = 0;
}

void Power::Gearing::ReloadParams(SimpleXMLTransfer* xml)
{
  int idx = xml->indexOfChild("gearing");
  
  if (idx >= 0)
  {
    SimpleXMLTransfer* it = xml->getChildAt(idx);
    i   = it->attributeAsDouble("i",   i);
    J_G = it->attributeAsDouble("J",   J);
    
    std::cout << "Gearing    : i=" << i << ", J=" << J_G << " kg m^2\n";
  }
}

double Power::Gearing::getJ()
{
  return(J_G + J*(i*i));
}

