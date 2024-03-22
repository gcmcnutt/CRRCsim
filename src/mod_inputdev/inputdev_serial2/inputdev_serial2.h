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
#ifndef TX_INTERFACE_SERIAL2_H
#define TX_INTERFACE_SERIAL2_H

#include "LoggerReader_ttyS.h"
#include "../inputdev.h"
#include "../../mod_misc/SimpleXMLTransfer.h"

/**
 * This is a very special interface, as it isn't useful to anyone but me (at
 * this moment, at least). My TX is able to send out its state via a serial
 * line in some special protocol. So this input-method decodes this protocol to
 * feed the values to crrcsim.
 * 
 * At the moment it only works for linux, but can easily be ported. Only the routines 
 * for reading the serial port need to be rewritten for others OSs.
 * 
 * If anyone is interested in using this interface (simple, as the UART is used),
 * I will of course write down some additional info about it. 
 * 
 * Jens Wilhelm Wulf, 14.08.2004
 */
class T_TX_InterfaceSerial2 : public T_TX_Interface
{
  public:
   T_TX_InterfaceSerial2();
   virtual ~T_TX_InterfaceSerial2();
   
   /**
    * Get input method
    */
   virtual int inputMethod() { return(T_TX_Interface::eIM_serial2); };
   
   /**
    * Initialize interface. Read further config data from a file, if necessary.
    */
   int init(SimpleXMLTransfer* config);
   
   /**
    * Write configuration back
    */
   virtual void putBackIntoCfg(SimpleXMLTransfer* config);
      
   /**
    * Set current input data. If some value is not available, the value 
    * is not overwritten.
    */
   void getInputData(TSimInputs* inputs);
   
  private:
   LoggerReader_ttyS* input;
   signed char        nAileron;
   signed char        nElevator;
   signed char        nRudder;
   signed char        nThrottle;
   signed char        nAileronTrim;
   signed char        nElevatorTrim;
   signed char        nRudderTrim;
   
   int                baudrate;
   std::string        device;
};

#endif
