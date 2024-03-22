/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2004, 2005, 2008 - Jens Wilhelm Wulf (original author)
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
#include "inputdev_serial2.h"

#include <stdio.h>

T_TX_InterfaceSerial2::T_TX_InterfaceSerial2()
{
#if DEBUG_TX_INTERFACE > 0
  printf("T_TX_InterfaceSerial2::T_TX_InterfaceSerial2()\n");
#endif  
  input = (LoggerReader_ttyS*)0;
}

T_TX_InterfaceSerial2::~T_TX_InterfaceSerial2()
{
#if DEBUG_TX_INTERFACE > 0
  printf("T_TX_InterfaceSerial2::~T_TX_InterfaceSerial2()\n");
#endif  
  if (input != (LoggerReader_ttyS*)0)
    delete input;
}

int T_TX_InterfaceSerial2::init(SimpleXMLTransfer* config)
{
#if DEBUG_TX_INTERFACE > 0
  printf("int T_TX_InterfaceSerial2::init(SimpleXMLTransfer* config)\n");
#endif  
  T_TX_Interface::init(config);
  
  baudrate = config->getInt("inputMethod.serial2.baudrate", 19200);
  device   = config->getString("inputMethod.serial2.device", "/dev/ttyS0");
  input = new LoggerReader_ttyS(200,
                                device.c_str(),
                                baudrate);
  if (input->isOK())
    return(0);
  else
  {
    errMsg = input->getErr();
    return(-1);
  }
}

void T_TX_InterfaceSerial2::putBackIntoCfg(SimpleXMLTransfer* config)
{
#if DEBUG_TX_INTERFACE > 0
  printf("int T_TX_InterfaceSerial2::putBackIntoCfg(SimpleXMLTransfer* config)\n");
#endif  
  T_TX_Interface::putBackIntoCfg(config);
  
  config->setAttributeOverwrite("inputMethod.serial2.device",   device);
  config->setAttributeOverwrite("inputMethod.serial2.baudrate", baudrate);  
}

void T_TX_InterfaceSerial2::getInputData(TSimInputs* inputs)
{
#if DEBUG_TX_INTERFACE > 1
  printf("void T_TX_InterfaceSerial2::getInputData(TSimInputs* inputs)\n");
#endif  
  // Read data
  const unsigned char* ptr;
  input->fetchData();
  while (input->hasFrames() > 0)
  {
    ptr = input->getFrame();
    
    signed char chVA = (((ptr[2] & 0x0F) << 2) | ((ptr[3] >> 6) & 0x3)) - 32;
    signed char chVB = (ptr[3] & 0x3F) - 32;
    
    switch (ptr[0])
    {
     case 0:
      nRudder       = chVB;
      nElevator     = chVA;
      break;
     case 1:
      nRudderTrim   = chVB;
      nThrottle     = chVA;
      break;
     case 2:
      nElevatorTrim = chVA;
    }
    input->nextFrame();
  }
  inputs->elevator = ((((int)nElevator)<<2) + nElevatorTrim)/320.0;
  // Aileron muss ein anderes Vorzeichen haben als Rudder, damit es gleich wirkt!
  inputs->rudder   = ((((int)nRudder)  <<2) + nRudderTrim  )/-320.0;
  inputs->aileron = -1*inputs->rudder; // weil ja fast alle Flieger nur auf aileron reagieren, auf rudder nicht!
  if (nThrottle > 0)
    inputs->throttle =  nThrottle/31.0;
  else
    inputs->throttle = 0;
}
