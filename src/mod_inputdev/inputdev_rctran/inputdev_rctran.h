/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2004, 2005, 2008 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2005 - Jan Reucker
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
#ifndef TX_INTERFACE_RCTRAN_H
#define TX_INTERFACE_RCTRAN_H

#include "../inputdev.h"
#include "../../mod_misc/SimpleXMLTransfer.h"

/**
 * default comment
 */
class T_TX_InterfaceRCTRAN : public T_TX_Interface
{
  public:
   T_TX_InterfaceRCTRAN();
   virtual ~T_TX_InterfaceRCTRAN();
   
   /**
    * Get input method
    */
   virtual int inputMethod() { return(T_TX_Interface::eIM_rctran); };
   
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
   
   /** \brief Get raw interface data as float values.
    *
    *  Fills the memory pointed to by <code>dest</code> with
    *  raw data values in float format
    *  \param dest Memory area to be filled with raw data. Make
    *              sure it can hold getNumAxes() values!
    */
   void getRawData(float *dest);
   
   /**
    * Get the number of axis supported by this interface.
    */
   int getNumAxes();
   
#ifdef WIN32
   unsigned long *rctran;
#else // WIN32
   volatile unsigned long long *rctran;  // Linux LPT rctran interface shmem
#endif
   float myFloatValues[8];
      
  private:
   float cpu_speed;
};

#endif
