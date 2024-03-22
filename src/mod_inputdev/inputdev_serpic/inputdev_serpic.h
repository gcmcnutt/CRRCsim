/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2008 - Jan Reucker (original author)
 *   Copyright (C) 2007, 2008 - Jens Wilhelm Wulf
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
#ifndef TX_INTERFACE_SERPIC
#define TX_INTERFACE_SERPIC

#include "../inputdev_serial/inputdev_serial.h"

/** \brief The serial PIC interface.
 *  
 *  This interface handles a transmitter connected to a
 *  serial port via an FMS-style "PIC" cable.
 *
 *  Interface hardware:
 *
 *  There are several schematics floating around the WWW.
 *  A good example can be found on
 *  http://myweb.absa.co.za/eric.brouwer/interface.htm
 *
 *  The interfaces are designed to work with the FMS r/c
 *  simulator which is unfortunately a Windows-only project
 *  (http://fms.pathbot.com). This simulator supports three
 *  flavours of interfaces: parallel, simple serial and
 *  PIC serial. The simple serial interface only works with
 *  OSes which allow direct polling of the serial ports,
 *  like DOS or Win98. The PIC serial interface is designed
 *  to work with any OS by independently transforming the
 *  r/c transmitter's PPM waveform into simple 8-bit values.
 *  Those values are then sent via the serial port to the
 *  computer. The start of a transmission is marked by a
 *  unique sync byte. After that "header" byte, each byte
 *  represents the value of one channel.
 *
 *  The serial transmission runs without flow
 *  control. To activate the interface's power supply, RTS
 *  has to be set while DTR must be cleared.
 *
 *  There are two speed variants of this interface:
 *  - 9600 baud, sync byte 0xF0 + channel count (0xF1 ... 0xF8)
 *  - 19200 baud, sync byte 0xFF
 *
 *  Some interface designs will allow you to switch between
 *  both modes. CRRCsim supports both, so choose whatever
 *  you like.
 */
class T_TX_InterfaceSerPIC: public T_TX_InterfaceSerial
{
  public:
    T_TX_InterfaceSerPIC();
    ~T_TX_InterfaceSerPIC();

    std::string getInterfaceName () { return "FMSPIC"; }

    int   init (SimpleXMLTransfer *config);
    void  putBackIntoCfg (SimpleXMLTransfer *config);
    void  processDataByte (unsigned char ucByte);
    int   inputMethod () { return T_TX_Interface::eIM_serpic; }
    int   getNumAxes() { return iSPIC_NumChannels; };
    int   getDeviceSpeed ();
    void  setDeviceSpeed (int speed);
  
  private:
    int iSyncState;
    int iSyncCounter;
    int iSPIC_NumChannels;        ///< How many channels are transmitted?
    int iSPIC_ButtonChannel;      ///< Does channel 0 contain button states?
    int iSPIC_ChanCount;          ///< which channel is to be received next?
    unsigned char ucSyncByte;     ///< Which value is used as SYNC marker?

    enum {WAIT_SYNC = 0, COUNT_CHANNELS, IN_SYNC, TRANSMITTER_ERROR};
};

#endif    // TX_INTERFACE_SERPIC
