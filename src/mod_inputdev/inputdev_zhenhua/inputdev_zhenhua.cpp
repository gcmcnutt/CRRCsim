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
#include "inputdev_zhenhua.h"

#define MARKER (0xEF)

T_TX_InterfaceZhenHua::T_TX_InterfaceZhenHua ()/*{{{*/
    :T_TX_InterfaceSerial (),
    nextChannel (-1)
{
#if DEBUG_TX_INTERFACE > 0
  printf ("T_TX_InterfaceZhenHua::T_TX_InterfaceZhenHua ()\n");
#endif
}
/*}}}*/

T_TX_InterfaceZhenHua::~T_TX_InterfaceZhenHua ()/*{{{*/
{
#if DEBUG_TX_INTERFACE > 0
  printf ("T_TX_InterfaceZhenHua::~T_TX_InterfaceZhenHua ()\n");
#endif
}
/*}}}*/

void T_TX_InterfaceZhenHua::processDataByte (unsigned char byte)/*{{{*/
{
  if (byte==MARKER)
  {
    // The marker byte will always reset the channel counter
    nextChannel=0;
  }
  else if (nextChannel>=0 && nextChannel<TX_MAXAXIS)
  {
    // Read up to TX_MAXAXIS channels
    setRawData (nextChannel, scaleValue (reverseByte (byte))-0.5);
    nextChannel++;
  }
  else
  {
    // Wait for the marker byte
    nextChannel=-1;
  }
}
/*}}}*/

uint8_t T_TX_InterfaceZhenHua::reverseByte (uint8_t data)/*{{{*/
{
  uint8_t result=0;

  for (int i=0; i<8; ++i)
  {
    result<<=1;
    if (data&1) result++;
    data>>=1;
  }

  return result;
}
/*}}}*/

float T_TX_InterfaceZhenHua::scaleValue (uint8_t data)/*{{{*/
{
  // Input range: 52...198
  return (data-52)/(float)(198-52);
}
/*}}}*/

int T_TX_InterfaceZhenHua::init (SimpleXMLTransfer *config)/*{{{*/
{
  int ret=T_TX_InterfaceSerial::init (config);

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
      cerr << "Serial interface initialization: " << getErrMsg () << endl;
      ret = 1;
    }
  }

  return ret;
}
/*}}}*/

