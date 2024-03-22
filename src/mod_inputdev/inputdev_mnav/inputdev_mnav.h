/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2006, 2008 - Todd Templeton (original author)
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
// Created 11/09/06 Todd R. Templeton <ttemplet@eecs.berkeley.edu>
// Based on tx_serial2.h

#ifndef TX_INTERFACE_MNAV_H
#define TX_INTERFACE_MNAV_H

#include "mnav.h"
#include "../inputdev.h"
#include "../../mod_misc/SimpleXMLTransfer.h"

class T_TX_InterfaceMNAV : public T_TX_Interface
{
  public:
   T_TX_InterfaceMNAV();
   virtual ~T_TX_InterfaceMNAV();
   
   /**
    * Get input method
    */
   virtual int inputMethod() { return(T_TX_Interface::eIM_mnav); };
   
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
   MNAV*              input;
   uint16_t           cnt_cmd[3];
   uint8_t            reverse;
   
   std::string        device;
};

#endif
