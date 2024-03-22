/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2004, 2005, 2008 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2005, 2009 - Jan Reucker
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
#include "inputdev_rctran.h"

#include "../../mod_misc/lib_conversions.h"

#ifdef linux
  #include "mbuff.h"
#endif

#include <stdio.h>

T_TX_InterfaceRCTRAN::T_TX_InterfaceRCTRAN()
{
}

T_TX_InterfaceRCTRAN::~T_TX_InterfaceRCTRAN()
{
  delete map;
  delete calib;
}

int T_TX_InterfaceRCTRAN::init(SimpleXMLTransfer* config)
{
  T_TX_Interface::init(config);

  cpu_speed = config->getDouble("inputMethod.rctran.cpu_speed", 456.507890);

#ifdef linux
  rctran = (volatile unsigned long long *)mbuff_attach("crrcsim",
                                                       sizeof(volatile unsigned long long) * 8);
#else
  rctran = 0;
#endif
  
  map = new T_AxisMapper(this, config, "inputMethod.rctran");
  calib = new T_Calibration(this, config, "inputMethod.rctran");

  if (!rctran)
  {
    errMsg = "Mapping shared memory for RC transmitter interface failed.\n";
    return(-1);
  }
  else
    return(0);
}

void T_TX_InterfaceRCTRAN::putBackIntoCfg(SimpleXMLTransfer* config)
{
  T_TX_Interface::putBackIntoCfg(config);

  config->setAttributeOverwrite("inputMethod.rctran.cpu_speed", doubleToString(cpu_speed));
  map->putBackIntoCfg(config);
}

void T_TX_InterfaceRCTRAN::getInputData(TSimInputs* inputs)
{
  getRawData(myFloatValues);
  CalibMixMapValues(inputs, myFloatValues);
}


void T_TX_InterfaceRCTRAN::getRawData(float *dest)
{
  int axes = getNumAxes();
  
  if (axes > TX_MAXAXIS)
  {
    axes = TX_MAXAXIS;
  }
  for (int i = 0; i < axes; i++)
  {
    // Actually this is all which is similar to what Jan Edward Kansky did
    // in his original rctran code:
    *(dest + i) = (float)rctran[i];
  }
}


int T_TX_InterfaceRCTRAN::getNumAxes()
{
  return 4;
}
