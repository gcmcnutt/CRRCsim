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
#ifndef TX_INTERFACEPPM_H
# define TX_INTERFACEPPM_H

# include "../inputdev.h"
# include "../../mod_misc/SimpleXMLTransfer.h"

# include <stdio.h>

/**
 * There already are some TX-interfaces implemented into crrcsim and therefore
 * lots of 'if' and things in the code. I wanted to add my own interface, but
 * I don't like 'if' that much, so I implemented a new scheme.
 * All those 'if' could be removed if the interfaces were ported to this scheme.
 * See tx_interface.h and the things in interface_serial2 for an example.
 *
 * Now every other input method uses this scheme, too.
 *
 * There is a pointer to a class T_TX_Interface in crrc_main, which is
 * initialized on startup. Depending on the configuration, it points to one of
 * the interface-classes. That's it, no more 'if'.
 *
 * Jens Wilhelm Wulf, 06.01.2005
 */
class T_TX_InterfacePPM : public T_TX_Interface
{
  public:
   T_TX_InterfacePPM();
   virtual ~T_TX_InterfacePPM();

   /**
    * Initialize interface. Read further config data from a file, if necessary.
    */
   virtual int init(SimpleXMLTransfer* config);

   /**
    * Set current input data. If some value is not available, the value
    * is not overwritten.
    */
   virtual void getInputData(TSimInputs* inputs);

   /** \brief Get raw interface data as float values.
    *
    *  Fills the memory pointed to by <code>dest</code> with
    *  raw data values in float format
    *  \param dest Memory area to be filled with raw data. Make
    *              sure it can hold getNumAxes() values!
    */
   virtual void getRawData(float *dest);
   
   /** \brief Get the name of the interface for the config file
    *
    *  Returns the name of the interface. It should be a single
    *  word, only lower-case characters. Every interface
    *  derived from this class must implement this method!
    */
   virtual const char * getConfigName() = 0;

   /**
    * Write configuration back
    */
   virtual void putBackIntoCfg(SimpleXMLTransfer* config);

  protected:
   /**
    * Channel values, range -1..1
    */
   float rc_channel_values[11];
};

#endif
