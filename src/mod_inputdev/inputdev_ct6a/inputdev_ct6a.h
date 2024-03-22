/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2009 - Nikolay Borisovich Eremeyev (original author)
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
#ifndef TX_INTERFACE_CT6A
#define TX_INTERFACE_CT6A

#include "../inputdev_serial/inputdev_serial.h"

/** This code is based on Zhen Hua interface code
 *
 ************************************************** *************************************
 * Description of Turborix 2.4Ghz Programmable Radio Gear Channel Data/Settings array's *
 ************************************************** *************************************

Baudrate is 115kbd
serial data : 8 bit, noparity, nohandshake

Channel data is send by transmitter approx every 20mSec.
Data burst duration (90uSec * 18 Bytes) is approx 1.6 mSec


Description of Channel data array ( 18 bytes )
------------------------------------------------------------------------------------------------------------------
index | description | Values
------------------------------------------------------------------------------------------------------------------
0 : header0 = 85
1 : header1 = 252
2 : high_byte of (ch1+1000)
3 : low_byte of (ch1+1000)
4 : high_byte of (ch2+1000)
5 : low_byte of (ch2+1000)
6 : high_byte of (ch3+1000)
7 : low_byte of (ch3+1000)
8 : high_byte of (ch4+1000)
9 : low_byte of (ch4+1000)
10: high_byte of (ch5+1000)
11: low_byte of (ch5+1000)
12: high_byte of (ch6+1000)
13: low_byte of (ch6+1000)
14: high_byte of 2044-(ch3+1000) !
15: low_byte of 2044-(ch3+1000) !
16: high_byte of checksum
17: low_byte of checksum


high_byte -> (Channelvalue + 1000) / 256
low_byte -> (Channelvalue + 1000) mod 256
checksum -> sum of bytes 2 to 15

The protocol complete description contains in a file protocol.txt.

***************************************************/
typedef struct
{
  unsigned short marker;
  unsigned short channel[6];
  unsigned short sum[2];
}  data_CT6A;

class T_TX_InterfaceCT6A: public T_TX_InterfaceSerial
{
  public:
    T_TX_InterfaceCT6A ();
    ~T_TX_InterfaceCT6A ();

    virtual int init (SimpleXMLTransfer *config);
    virtual std::string getInterfaceName ()
    {
      return "CT6A";
    }
    virtual void processDataByte (unsigned char byte);
    int inputMethod ()
    {
      return T_TX_Interface::eIM_CT6A;
    }
    int getNumAxes()
    {
      return 6;
    }

  protected:
    float scaleValue (unsigned short data);
  private:
    int nextChannel;
    data_CT6A data;
    char count;
};


#endif    // TX_INTERFACE_CT6A

