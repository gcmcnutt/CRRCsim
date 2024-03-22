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

#include "inputdev_ct6a.h"

T_TX_InterfaceCT6A::T_TX_InterfaceCT6A ()/*{{{*/
    :T_TX_InterfaceSerial (),
    nextChannel (-1)
{
  count = 0;
#if DEBUG_TX_INTERFACE > 0
  printf ("T_TX_InterfaceCT6A::T_TX_InterfaceCT6A ()\n");
#endif
}
/*}}}*/

T_TX_InterfaceCT6A::~T_TX_InterfaceCT6A ()/*{{{*/
{
#if DEBUG_TX_INTERFACE > 0
  printf ("T_TX_InterfaceCT6A::~T_TX_InterfaceCT6A ()\n");
#endif
}
/*}}}*/

void T_TX_InterfaceCT6A::processDataByte (unsigned char byte)/*{{{*/
{
  switch (count)
  {
    case 0:
      if (byte == 0x55)
      {
        data.marker = byte << 8;
        count++;
      }
      break;
    case 1:
      if (byte== 0xFC)
      {
        data.marker = data.marker + byte;
        count++;
        nextChannel=0;
      }
      else
      {
        count = 0;
        nextChannel=-1;
      }
      break;
    case 2:
    case 4:
    case 6:
    case 8:
    case 10:
    case 12:
      data.channel[nextChannel] = byte << 8;
      count++;
      break;
    case 3:
    case 5:
    case 7:
    case 9:
    case 11:
    case 13:
      data.channel[nextChannel] = data.channel[nextChannel] + byte;
      setRawData (nextChannel, scaleValue (data.channel[nextChannel])-0.5);
      count++;
      nextChannel++;
      break;
    case 14:
    case 16:
      data.sum[(count-14)/2] = byte << 8;
      count++;
      break;
    case 15:
    case 17:
      data.sum[(count-14)/2] = data.sum[(count-14)/2] + byte;
      count++;
      break;
  }
  count = count % 18;
  /*}}}*/
}
float T_TX_InterfaceCT6A::scaleValue (unsigned short data)/*{{{*/
{
  // Input range: 52...198
  return (data-1000)/(float)(2000-1000);
}
/*}}}*/

int T_TX_InterfaceCT6A::init (SimpleXMLTransfer *config)/*{{{*/
{
  int ret = T_TX_InterfaceSerial::init (config);

  if (ret == 0)
  {
    // initialized successfully, now turn on the power supply for the
    // interface hardware (careful, could throw an exception)
    try
    {
      setRts (true);
      setDtr (false);
    }
    catch (CharDevice::ConfigureDeviceException e)
    {
      setErrMsg ("Setting Rts/Dtr states failed.");
      std::cerr << "CT6A interface initialization: " << getErrMsg () << std::endl;
      ret = 1;
    }
  }

  return ret;
}
/*}}}*/

