/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2001 - Jan Edward Kansky (original author)
 *   Copyright (C) 2004, 2005, 2008 - Jens Wilhelm Wulf
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
#ifndef TX_INTERFACE_Parallel_H
#define TX_INTERFACE_Parallel_H

#include "../inputdev_PPM/inputdev_PPM.h"
#include "../../mod_misc/SimpleXMLTransfer.h"

/**
 * default comment
 */
class T_TX_InterfaceParallel : public T_TX_InterfacePPM
{
  public:
   T_TX_InterfaceParallel();
   virtual ~T_TX_InterfaceParallel();
   
   /**
    * Get input method
    */
   int inputMethod() { return(T_TX_Interface::eIM_parallel); };
   
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
   void calibrate_parallel_interface();
   int init_parallel_interface(int lpt);
   void get_data_from_parallel_interface(float *rc_channel_values);
   
   void wait_until_interface_is_idle(unsigned char *data);
   void wait_until_data_ready(unsigned char *data);
   void acknowledge_got_data();
   void ask_for_data();
   
   unsigned int LPT_DATA;
   unsigned int LPT_STATUS;
   unsigned int LPT_CONTROL;
     
   /**
    * Number of ticks per second of the mcpu
    */
   float lpt_calibration_factor;
   
   /**
    * Number of parallel port
    */
   int nParPortNum;

   const char *cname;  ///< name in the config file
};

#endif
