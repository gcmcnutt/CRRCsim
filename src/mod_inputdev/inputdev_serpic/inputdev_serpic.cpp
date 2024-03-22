/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2007, 2008 - Jan Reucker (original author)
 *   Copyright (C) 2007, 2008 - Jens Wilhelm Wulf
 *   Copyright (C) 2008 - Todd Templeton
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
#include "inputdev_serpic.h"

#define DEFAULT_SYNC_BYTE_19200       (0xFF)
#define DEFAULT_SYNC_BYTE_9600        (0xF0)
#define DEFAULT_SYNC_BYTE             DEFAULT_SYNC_BYTE_19200
#define DEFAULT_BUTTON_CHANNEL_19200  (0)
#define DEFAULT_BUTTON_CHANNEL_9600   (1)
#define DEFAULT_BUTTON_CHANNEL        DEFAULT_BUTTON_CHANNEL_19200


/**
 *  The constructor.
 *
 */
T_TX_InterfaceSerPIC::T_TX_InterfaceSerPIC()
  : T_TX_InterfaceSerial(),
    iSyncState(WAIT_SYNC), iSyncCounter(0),
    iSPIC_NumChannels(0), iSPIC_ButtonChannel(DEFAULT_BUTTON_CHANNEL),
    iSPIC_ChanCount(0), ucSyncByte(DEFAULT_SYNC_BYTE)
{
#if DEBUG_TX_INTERFACE > 0
  std::cout << "T_TX_InterfaceSerPIC::T_TX_InterfaceSerPIC ()\n";
#endif
}


/**
 *  The destructor.
 *
 */
T_TX_InterfaceSerPIC::~T_TX_InterfaceSerPIC()
{
#if DEBUG_TX_INTERFACE > 0
  std::cout << "T_TX_InterfaceSerPIC::~T_TX_InterfaceSerPIC ()\n";
#endif
}


/**
 *  Initialize the interface.
 *
 *  The base class handles all hardware initialization for us, so we only
 *  have to set up the correct control line states to power the interface.
 */
int T_TX_InterfaceSerPIC::init (SimpleXMLTransfer *config)
{
#if DEBUG_TX_INTERFACE > 0
  std::cout << "T_TX_InterfaceSerPIC::init ()\n";
#endif
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
      cerr << "Serial interface initialization: " << getErrMsg () << endl;
      ret = 1;
    }

    int default_sync_byte = DEFAULT_SYNC_BYTE_19200;
    int default_button_channel = DEFAULT_BUTTON_CHANNEL_19200;
    if (T_TX_InterfaceSerial::getBaudRate() == 9600)
    {
      default_sync_byte = DEFAULT_SYNC_BYTE_9600;
      default_button_channel = DEFAULT_BUTTON_CHANNEL_9600;
    }
    // read sync and button byte settings from config file
    SimpleXMLTransfer *port = config->getChild(getXmlChildName() + ".port", true);
    ucSyncByte              = port->attributeAsInt("sync", default_sync_byte);
    iSPIC_ButtonChannel     = port->attributeAsInt("button_channel", default_button_channel);
#if DEBUG_TX_INTERFACE > 0
    std::cout << "  Configured sync byte: 0x" << std::hex << int(ucSyncByte) << std::dec;
    std::cout << ", " << std::string((iSPIC_ButtonChannel == 0) ? "no" : "has") << " button channel";
    std::cout << std::endl;
#endif
  }

  return ret;
}


void T_TX_InterfaceSerPIC::putBackIntoCfg (SimpleXMLTransfer *config)/*{{{*/
{
#if DEBUG_TX_INTERFACE > 0
  std::cout << "T_TX_InterfaceSerPIC::putBackIntoCfg(SimpleXMLTransfer *config)" << std::endl;
#endif

  // Store the port settings
  T_TX_InterfaceSerial::putBackIntoCfg(config);

  // Store additional settings
  SimpleXMLTransfer *port = config->getChild(getXmlChildName() + ".port", true);
  port->setAttributeOverwrite ("sync", ucSyncByte);
  port->setAttributeOverwrite ("button_channel", iSPIC_ButtonChannel);
}


/**
 *  Process the next data byte.
 *
 *  This method processes the next byte that was received on the serial
 *  port.
 *
 *  \param byte   The byte to be processed.
 */
void T_TX_InterfaceSerPIC::processDataByte (unsigned char ucByte)
{
  static int chancount;

  switch (iSyncState)
  {
    default:
    case WAIT_SYNC:
      // detect some sync bytes to make sure the interface
      // sends a valid stream of bytes, not any startup trash
      if (ucByte >= ucSyncByte)
      {
        iSyncCounter++;
      }
      if (iSyncCounter > 4)
      {
        iSyncState = COUNT_CHANNELS;
        iSPIC_NumChannels = 0;
        chancount = 0;
        #if DEBUG_TX_INTERFACE > 0
        printf("T_TX_InterfaceSerPIC: detected sync bytes: 0x%02X\n", ucByte);
        printf("T_TX_InterfaceSerPIC state machine set to COUNT_CHANNELS\n");
        #endif
      }
      break;
      

    case COUNT_CHANNELS:
      iSyncCounter = 0;
      if (ucByte < ucSyncByte)
      {
        chancount++;
      }
      else
      {
        // received next sync byte, end of sync
        iSyncState = IN_SYNC;
        int tmp = (chancount - iSPIC_ButtonChannel);
        #if DEBUG_TX_INTERFACE > 0
        printf("T_TX_InterfaceSerPIC: detected %d channels\n", tmp);
        printf("T_TX_InterfaceSerPIC state machine set to IN_SYNC\n");
        #endif
        if (tmp > TX_MAXAXIS)
        {
          fprintf(stdout, 
                  "T_TX_InterfaceSerPIC: This version of T_TX_InterfaceSerPIC only \n            supports %d channels, will only use 0-7!\n",
                  TX_MAXAXIS);
          tmp = TX_MAXAXIS;
          //~ iSyncState = TRANSMITTER_ERROR;
        }
        iSPIC_NumChannels = tmp;
      }
      break;
      
    case IN_SYNC:
      if (ucByte >= ucSyncByte)
      {
        /* received SYNC byte */
        iSPIC_ChanCount = 0;
      }
      else
      {
        int current_channel = (iSPIC_ChanCount - iSPIC_ButtonChannel);
        if ((current_channel >= 0) && (current_channel < TX_MAXAXIS))
        {
          setRawData(current_channel, (float)((float)ucByte/175.0 - 0.5));
        }
        iSPIC_ChanCount++;
      }
      break;

    case TRANSMITTER_ERROR:
      break;
  }
}


/**
 *  Get the device speed.
 *  
 *  This method overrides T_TX_InterfaceSerial::getDeviceSpeed().
 *  The reason for this is that T_TX_InterfaceSerPIC only supports
 *  two speeds, 9600 baud and 19200 baud. Therefore the GUI only
 *  offers these two values. If the translation from the speed value to the
 *  GUI's combo box item is done in the GUI, the GUI needs to know too much
 *  about this interface, so it's much cleaner to do the translation here.
 */
int T_TX_InterfaceSerPIC::getDeviceSpeed()
{
  int speed;

  // speed as reported by the base class
  int speed_serial = T_TX_InterfaceSerial::getDeviceSpeed();
  
  if (speed_serial == 6)
  {
    // 9600 baud
    speed = 0;
  }
  else
  {
    // 19200 baud, default
    speed = 1;
  }
  return speed;
}

/**
 *  Set the device speed.
 *
 *  This method overrides T_TX_InterfaceSerial::setDeviceSpeed(int).
 *  The reason for this is that T_TX_InterfaceSerPIC only supports
 *  two speeds, 9600 baud and 19200 baud. Therefore the GUI only
 *  offers these two values. If the translation from the speed value to the
 *  GUI's combo box item is done in the GUI, the GUI needs to know too much
 *  about this interface, so it's much cleaner to do the translation here.
 */
void T_TX_InterfaceSerPIC::setDeviceSpeed(int speed)
{
  int speed_serial;
  
  if (speed == 0)
  {
    // 9600 baud
    speed_serial = 6;
    ucSyncByte = 0xF0;
    iSPIC_ButtonChannel = 1;
  }
  else
  {
    // 19200 baud, default
    speed_serial = 7;
    ucSyncByte = 0xFF;
    iSPIC_ButtonChannel = 0;
  }
  T_TX_InterfaceSerial::setDeviceSpeed(speed_serial);
}

