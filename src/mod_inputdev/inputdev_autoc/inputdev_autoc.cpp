/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2006, 2008 - Todd Templeton (original author)
 *   Copyright (C) 2008 - Jan Reucker
 *   Copyright (C) 2008 - Jens Wilhelm Wulf
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

#include "../../crrc_main.h"
#include "../../global.h"
// #include "../../aircraft.h"
// #include "../../SimStateHandler.h"
// #include "../../mod_fdm/fdm.h"
#include "inputdev_autoc.h"

#include <stdio.h>

T_TX_InterfaceAUTOC::T_TX_InterfaceAUTOC()
{
#if DEBUG_TX_INTERFACE > 0
  printf("T_TX_InterfaceAUTOC::T_TX_InterfaceAUTOC()\n");
#endif  
}

T_TX_InterfaceAUTOC::~T_TX_InterfaceAUTOC()
{
#if DEBUG_TX_INTERFACE > 0
  printf("T_TX_InterfaceAUTOC::~T_TX_InterfaceAUTOC()\n");
#endif  
}

int T_TX_InterfaceAUTOC::init(SimpleXMLTransfer* config)
{
#if DEBUG_TX_INTERFACE > 0
  printf("int T_TX_InterfaceAUTOC::init(SimpleXMLTransfer* config)\n");
#endif  
  char devicestr[100];

  T_TX_Interface::init(config);
  
  device   = config->getString("inputMethod.autoc.device", "udpserver,127.0.0.1/0.0.0.0,9002");
  strncpy(devicestr, device.c_str(), 100); devicestr[99] = '\0';
  
  return(0);
}

void T_TX_InterfaceAUTOC::putBackIntoCfg(SimpleXMLTransfer* config)
{
#if DEBUG_TX_INTERFACE > 0
  printf("int T_TX_InterfaceAUTOC::putBackIntoCfg(SimpleXMLTransfer* config)\n");
#endif  
  T_TX_Interface::putBackIntoCfg(config);
  
  config->setAttributeOverwrite("inputMethod.autoc.device",   device);  
}

void T_TX_InterfaceAUTOC::getInputData(TSimInputs* inputs)
{
#if DEBUG_TX_INTERFACE > 1
  printf("void T_TX_InterfaceAUTOC::getInputData(TSimInputs* inputs)\n");
#endif  
  {
    inputs->throttle = 1.0;
    inputs->elevator = -0.3;
  }
}
