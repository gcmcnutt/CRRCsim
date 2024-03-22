/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2007, 2008 - Martin Herrmann (original author)
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
#ifndef TX_INTERFACE_ZHENHUA
#define TX_INTERFACE_ZHENHUA

#include "../inputdev_serial/inputdev_serial.h"

/** \brief The Zhen Hua interface
 *
 *  This interfaces handles the Zhen Hua 5 Byte protocol. This is the protocol
 *  used by the Walkera Dragonfly 4 channal transmitter.
 *
 *  The serial transmission runs without flow control.
 *  To activate the interface, RTS has to be set.
 */

class T_TX_InterfaceZhenHua: public T_TX_InterfaceSerial
{
  public:
    T_TX_InterfaceZhenHua ();
    ~T_TX_InterfaceZhenHua ();

    virtual int init (SimpleXMLTransfer *config);
    virtual std::string getInterfaceName ()
    {
      return "zhenhua";
    }
    virtual void processDataByte (unsigned char byte);
    int inputMethod ()
    {
      return T_TX_Interface::eIM_zhenhua;
    }
    int getNumAxes()
    {
      return 4;
    }

  protected:
    float scaleValue (uint8_t data);
    uint8_t reverseByte (uint8_t data);

  private:
    int nextChannel;

};


#endif    // TX_INTERFACE_SERPIC

