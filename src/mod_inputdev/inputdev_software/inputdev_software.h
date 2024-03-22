/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2008 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2005, 2008, 2010 - Jan Reucker
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
#ifndef TX_INTERFACE_SOFTWARE_H
# define TX_INTERFACE_SOFTWARE_H

# include "../inputdev.h"
# include "../../mod_misc/SimpleXMLTransfer.h"

/** \brief The software pseudo-Tx interface
 *
 *  This interface handles joysticks, the mouse and the
 *  keyboard as a replacement for a "real" transmitter.
 *  Note: transmitters with a USB interface will also
 *  use this interface as they usually emulate a joystick.
 */
class T_TX_InterfaceSoftware : public T_TX_Interface
{
  public:
    T_TX_InterfaceSoftware(int method, int axesnum);
    virtual ~T_TX_InterfaceSoftware();
    
   /**
    * Get input method
    */
    virtual int inputMethod() { return(input_method); };
    
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

   /** \brief Get number of axis.
    *
    *  Returns the number of axis for the current interface.
    */
    int getNumAxes() { return(numberOfAxes); };

   /** \brief Get raw interface data as float values.
    *
    *  Fills the memory pointed to by <code>dest</code> with
    *  raw data values in float format
    *  \param dest Memory area to be filled with raw data. Make
    *              sure it can hold getNumAxes() values!
    */
    void getRawData(float *dest);

   /**
    * Set the raw value for an axis.
    * \param axis axis number
    * \param x value
    */
    void setAxis(int axis, const float x);
    
    static int getDeviceList(std::vector<std::string>& Devices);

  private:
    float input_raw_data[TX_MAXAXIS];
    
    int input_method;
    int numberOfAxes;
};

#endif
