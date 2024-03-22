/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2004, 2005, 2008 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2005, 2009 - Jan Reucker
 *   Copyright (C) 2005 - Joel Lienard
 *   Copyright (C) 2005 - Lionel Cailler
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
#include "inputdev_PPM.h"

T_TX_InterfacePPM::T_TX_InterfacePPM()
{
#if DEBUG_TX_INTERFACE > 0
  printf("T_TX_InterfacePPM::T_TX_InterfacePPM()\n");
#endif  
}

T_TX_InterfacePPM::~T_TX_InterfacePPM()
{
#if DEBUG_TX_INTERFACE > 0
  printf("T_TX_InterfacePPM::~T_TX_InterfacePPM()\n");
#endif 
  delete calib;
  delete map;
}

int T_TX_InterfacePPM::init(SimpleXMLTransfer* config)
{
#if DEBUG_TX_INTERFACE > 0
  printf("int T_TX_InterfacePPM::init(SimpleXMLTransfer* config)\n");
#endif
  std::string name = getConfigName();
  std::string childname = "inputMethod." + name;
  
  calib = new T_Calibration(this, config, childname);
  map   = new T_AxisMapper(this, config, childname);

  return(T_TX_Interface::init(config));
}

void T_TX_InterfacePPM::getInputData(TSimInputs* inputs)
{
#if DEBUG_TX_INTERFACE > 1
  printf("int T_TX_InterfacePPM::getInputData(TSimInputs* inputs)\n");
#endif
  CalibMixMapValues(inputs, rc_channel_values);
}


void T_TX_InterfacePPM::getRawData(float *dest)
{
  int axes = getNumAxes();
  if (axes > TX_MAXAXIS)
  {
    axes = TX_MAXAXIS;
  }
  for (int i = 0; i < axes; i++)
  {
    *(dest + i) = rc_channel_values[i];
  }
}


void T_TX_InterfacePPM::putBackIntoCfg(SimpleXMLTransfer* config)
{
  calib->putBackIntoCfg(config);
  map->putBackIntoCfg(config);
  
  T_TX_Interface::putBackIntoCfg(config);
}
