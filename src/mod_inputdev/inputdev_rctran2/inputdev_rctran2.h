/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2008 - Jens Wilhelm Wulf (original author)
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
#ifndef TX_INTERFACE_RCTRAN2_H
#define TX_INTERFACE_RCTRAN2_H

#include <string>
#include <stdlib.h>

#include "../inputdev_PPM/inputdev_PPM.h"
#include "../../mod_misc/SimpleXMLTransfer.h"

/**
 * This interface reads channel values from a file (default: /proc/rctran).
 * The file contains one line with eight integer values in microseconds, separated
 * by a comma.
 */
class T_TX_Interface_RCTran2 : public T_TX_InterfacePPM
{
  public:
   T_TX_Interface_RCTran2();
   virtual ~T_TX_Interface_RCTran2();
   
   /**
    * Get input method
    */
   int inputMethod() { return(T_TX_Interface::eIM_rctran2); };
   
   /**
    * Initialize interface. Read further config data from a file, if necessary.
    */
   int init(SimpleXMLTransfer* config);
   
   /**
    * Set current input data. If some value is not available, the value 
    * is not overwritten.
    */
   void getInputData(TSimInputs* inputs);

   /**
    * Write configuration back
    */
   virtual void putBackIntoCfg(SimpleXMLTransfer* config);

   /** \brief Get the name of the interface for the config file
    *
    *  Returns the name of the interface. It should be a single
    *  word, only lower-case characters.
    */
   const char * getConfigName() {return cname;};

  private:
   /**
    * Path of the file to read values from
    */
   std::string  filenameToReadFrom;
   
   /**
    * file descriptor
    */
   FILE*        rctran;
  
   const char *cname;  ///< name in the config file
};

#endif
