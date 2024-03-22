/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2008 - Martin Herrmann (original author)
 *   Copyright (C) 2008, 2009 - Jan Reucker
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
#include <crrc_config.h>
#include <cstdio>
#include <plib/ul.h>

#include "inputdev_serial.h"
#include "../../mod_misc/lib_conversions.h"
#include "../../global.h"
#include <sstream>
//#include "../../mod_chardevice/chardevicebase.h"

#ifdef WIN32
#include <tchar.h>
#endif

// Remaining problems:
//   - crash when non-existent port is opened
//   - connect USB, choose USB, save, quit, disconnect USB, start ==> crash?

// TODO Kalibrierproblem

// TODO 110 Baud eingestellt nach Fehler

#ifdef WIN32
#define DEFAULT_PORT_NAME "COM1"
#else
#define DEFAULT_PORT_NAME "/dev/ttyS0"
#endif

#if DEBUG_TX_INTERFACE > 0
#define DEBUG(...) printf (__VA_ARGS__)
#else
#define DEBUG()
#endif

const int T_TX_InterfaceSerial::nNumBaudrates = 11;
const char *T_TX_InterfaceSerial::baudRates[] =
  {    "110",  "300",   "600",  "1200",  "2400",
       "4800", "9600", "19200", "38400", "57600",
       "115200",   NULL
  };
const int T_TX_InterfaceSerial::anBaudRates[] =
  {      110,    300,     600,    1200,    2400,
         4800,   9600,   19200,   38400,   57600,
         115200,     -1
  };


// ******************
// ** Construction **
// ******************

T_TX_InterfaceSerial::T_TX_InterfaceSerial ()/*{{{*/
    :charDevice (NULL), portName (""), baudRate (0)
{
  DEBUG ("T_TX_InterfaceSerial::T_TX_InterfaceSerial ()\n");

  mixer=new T_TX_Mixer (this);
  map=new T_AxisMapper (this);
  calib=new T_Calibration (this);

#if DEBUG_TX_INTERFACE>0
  dbgbufferidx=0;
#endif
}
/*}}}*/

T_TX_InterfaceSerial::~T_TX_InterfaceSerial ()/*{{{*/
{
  DEBUG("T_TX_InterfaceSerial::~T_TX_InterfaceSerial()\n");

  delete mixer;
  delete map;
  delete calib;
  delete charDevice;
}
/*}}}*/

// *******************
// ** Configuration **
// *******************

int T_TX_InterfaceSerial::init (SimpleXMLTransfer *config)/*{{{*/
{
  DEBUG ("int T_TX_InterfaceSerial::init (SimpleXMLTransfer *config)\n");

  // Initialize the port settings
  SimpleXMLTransfer *port=config->getChild (getXmlChildName ()+".port", true);
  portName = port->getString ("name", DEFAULT_PORT_NAME);
  baudRate = port->getInt ("baudrate", getDefaultBaudRate ());
  // Initialize the subsystems
  if (mixer->init (config, getXmlChildName ())!=0)
  {
    setErrMsg("Mixer initialization failed.");
    return 1;
  }
  calib->init (config, getXmlChildName ());
  map->init (config, getXmlChildName ());

  try
  {
    // Try-Methode?
    ostringstream options;
    options << portName << "," << baudRate;
    string optionString=options.str ();
    cout << "Opening the serial port with option string " << optionString << endl;
    charDevice=new SerialCharDevice (optionString.c_str (), false);
  }
  catch (CharDevice::ConfigureDeviceException e)
  {
    setErrMsg ("The device could not be configured.");
    cerr << "Serial interface initialization: " << getErrMsg () << endl;
    return 1;
  }

  cout << "Serial interface initialization: OK" << endl;
  return 0;
}
/*}}}*/

void T_TX_InterfaceSerial::putBackIntoCfg (SimpleXMLTransfer *config)/*{{{*/
{
  DEBUG("int T_TX_InterfaceSerial::putBackIntoCfg (SimpleXMLTransfer *config)\n");

  // Store the port settings
  SimpleXMLTransfer *port=config->getChild (getXmlChildName ()+".port", true);
  port->setAttributeOverwrite ("name", portName);
  port->setAttributeOverwrite ("baudrate", baudRate);

  // Store the subsystems
  if (mixer) mixer->putBackIntoCfg (config);
  if (map)   map  ->putBackIntoCfg (config);
  if (calib) calib->putBackIntoCfg (config);
}
/*}}}*/

std::string T_TX_InterfaceSerial::getDeviceName ()/*{{{*/
{
  return portName;
}
/*}}}*/

void T_TX_InterfaceSerial::setDeviceSpeed (int speed)/*{{{*/
{
  if ((speed >= 0) && (speed < nNumBaudrates))
  {
    baudRate = anBaudRates[speed];
  }
  else
  {
    baudRate = getDefaultBaudRate();
  }
}
/*}}}*/

int T_TX_InterfaceSerial::baudRateToSpeed (int br)/*{{{*/
{
  int nSpeed;

  switch (br)
  {
  case 110:
    nSpeed = 0;
    break;

  case 300:
    nSpeed = 1;
    break;

  case 600:
    nSpeed = 2;
    break;

  case 1200:
    nSpeed = 3;
    break;

  case 2400:
    nSpeed = 4;
    break;

  case 4800:
    nSpeed = 5;
    break;

  case 9600:
    nSpeed = 6;
    break;

  case 19200:
    nSpeed = 7;
    break;

  case 38400:
    nSpeed = 8;
    break;

  case 57600:
    nSpeed = 9;
    break;

  case 115200:
    nSpeed = 10;
    break;

  default:
    nSpeed = -1;
    break;
  }

  return nSpeed;
}
/*}}}*/

int T_TX_InterfaceSerial::getDeviceSpeed ()/*{{{*/
{
  int speed = baudRateToSpeed (baudRate);
  if (speed < 0)
    speed = baudRateToSpeed (getDefaultBaudRate ());
  return speed;
}
/*}}}*/

void T_TX_InterfaceSerial::setDeviceName (std::string devname)/*{{{*/
{
  portName = devname;
}
/*}}}*/

void T_TX_InterfaceSerial::setDtr (bool dtr)/*{{{*/
{
  if (dtr)
    charDevice->write_output_pins (SerialCharDevice::WriteOutputPinUnchanged, SerialCharDevice::WriteOutputPinHigh); // RTS, DTR
  else
    charDevice->write_output_pins (SerialCharDevice::WriteOutputPinUnchanged, SerialCharDevice::WriteOutputPinLow); // RTS, DTR
}
/*}}}*/

void T_TX_InterfaceSerial::setRts (bool rts)/*{{{*/
{
  if (rts)
    charDevice->write_output_pins (SerialCharDevice::WriteOutputPinHigh, SerialCharDevice::WriteOutputPinUnchanged); // RTS, DTR
  else
    charDevice->write_output_pins (SerialCharDevice::WriteOutputPinLow, SerialCharDevice::WriteOutputPinUnchanged); // RTS, DTR
}
/*}}}*/


// *****************
// ** Data access **
// *****************

void T_TX_InterfaceSerial::getInputData (TSimInputs *inputs)/*{{{*/
{
  // Read serial data and update rawData[]
  readSerialData ();

  CalibMixMapValues(inputs, rawData);
}
/*}}}*/

void T_TX_InterfaceSerial::getRawData (float *dest)/*{{{*/
{
  // Read serial data and update rawData[]
  readSerialData ();

  int numAxes=getNumAxes ();
  if (numAxes > TX_MAXAXIS)
  {
    numAxes = TX_MAXAXIS;
  }
  for (int i = 0; i < numAxes; ++i)
  {
    *(dest + i) = rawData[i];
  }
}
/*}}}*/


// *********
// ** I/O **
// *********

/**
 * Reads one byte from the serial interface. Only this method should be used
 * for reading serial data.
 * \return -1 if no more bytes available, the byte else
 */
int T_TX_InterfaceSerial::getSerialByte ()/*{{{*/
{
  unsigned char buffer;

  if (charDevice->read (&buffer, 1)<1)
    return -1;

#if DEBUG_TX_INTERFACE > 0
  debugHandler(buffer);
#endif

  return buffer;
}
/*}}}*/

void T_TX_InterfaceSerial::readSerialData ()/*{{{*/
{
  while (true)
  {
    int data=getSerialByte ();
    if (data<0) return;

    processDataByte (data);
  }
}
///*}}}*/

void T_TX_InterfaceSerial::setRawData (int num, float data)/*{{{*/
{
// DEBUG ("setting %d to %f\n", num, data);

  if (num>=0 && num<TX_MAXAXIS)
    rawData[num]=data;
}
/*}}}*/


#if DEBUG_TX_INTERFACE > 0
/**
 *  Generate some interesting debugging output.
 *
 *  Dump the byte stream received from the serial port to stdout.
 *  If DEBUG_TX_INTERFACE is set to 1, DBGBUFFERSIZE bytes are
 *  recorded once at startup and then dumped to stdout. If
 *  DEBUG_TX_INTERFACE is set to 2, the byte stream is continously
 *  dumped to stdout in chunks of DBGBUFFERSIZE bytes.
 *
 *  \param ucByte   the byte to be logged
 */
void T_TX_InterfaceSerial::debugHandler(unsigned char ucByte)
{
  if (dbgbufferidx < DBGBUFFERSIZE)
  {
    dbgbuffer[dbgbufferidx++] = ucByte;
  }
  else if (dbgbufferidx == DBGBUFFERSIZE)
  {
#if DEBUG_TX_INTERFACE == 1
    dbgbufferidx++;
#elif DEBUG_TX_INTERFACE == 2
    dbgbufferidx = 0;
#endif
    for (int i = 0; i < DBGBUFFERSIZE; i++)
    {
      if (i % 32 == 0)
      {
        printf("\n");
      }
      printf("%02X ", dbgbuffer[i]);
    }
    printf("\n");
  }
}
#endif // DEBUG_TX_INTERFACE > 0

/**
 * \brief Generate a list of serial ports
 *
 * This method generates a list of serial ports. Depending on the
 * OS, this can be done by querying the registry (on Windows) or
 * looking at the /dev hierarchy (Linux, MacOSX).
 *
 * \param   SerialPorts   Vector to be filled with the list of port names
 * \return  number of ports in the list (equivalent to SerialPorts.size())
 */
int T_TX_InterfaceSerial::getDeviceList(std::vector<std::string>& SerialPorts)
{
  // clear any possible previous content of SerialPorts
  SerialPorts.erase(SerialPorts.begin(), SerialPorts.end());

#ifdef WIN32
  // --- Windows implementation -----------------------------------------------
  TCHAR acValue[MAX_PATH];
  BYTE  abData[MAX_PATH];

  HKEY  hKey        = NULL;
  DWORD dwDataType  = 0;
  DWORD dwIndex     = 0;

  
  // open the registry key containing all active serial ports
  LONG lRet = ::RegOpenKeyEx( HKEY_LOCAL_MACHINE,
                              _T("HARDWARE\\DEVICEMAP\\SERIALCOMM"),
                              0,
                              KEY_READ,
                              &hKey);
  if(ERROR_SUCCESS != lRet)
  {
    // Error handling
    std::cerr << "Unable to open registry key HKEY_LOCAL_MACHINE\\HARDWARE\\DEVICEMAP\\SERIALCOMM" << std::endl;
  }
  else
  {
    // Get a list of all subkeys
    do
    {
      DWORD dwValueSize = MAX_PATH;
      DWORD dwDataSize = MAX_PATH;
      lRet = ::RegEnumValue(hKey,
                            dwIndex,
                            acValue,
                            &dwValueSize,
                            NULL,
                            &dwDataType,
                            abData,
                            &dwDataSize);
      if ((lRet == ERROR_SUCCESS) && (dwDataType == REG_SZ))
      {
        SerialPorts.push_back(std::string((char *)abData));
      }
      dwIndex++;
    } while (lRet == ERROR_SUCCESS);
    ::RegCloseKey(hKey);
  }
#else
  // --- implementation for other OSes ----------------------------------------
  #ifdef OLD_WAY
  const char* serialDevs[] = {"/dev/ttyS0", "/dev/ttyS1",
                              "/dev/ttyS2", "/dev/ttyS3",
                              "/dev/ttyUSB0", "/dev/ttyUSB1",
                              "/dev/ttyUSB2", "/dev/ttyUSB3",
                              NULL};
  const char** ptr;
  
  for (ptr = serialDevs; *ptr != NULL; ptr++)
  {
    SerialPorts.push_back(std::string(*ptr));
  }
  #else
  // look for some well-known names in /dev/
  std::vector<std::string> serialDevs;
  std::string sPath = "/dev";
  serialDevs.push_back("ttyS");
  serialDevs.push_back("ttyUSB");
  
  ulDir *dir = ulOpenDir(sPath.c_str());
  if (dir != NULL)
  {
    ulDirEnt *entry;
    
    do
    {
      entry = ulReadDir(dir);
      if (entry != NULL)
      {
        if (!entry->d_isdir)
        {
          std::vector<std::string>::iterator it;
          std::string dev = entry->d_name;
          for (it = serialDevs.begin(); it != serialDevs.end(); it++)
          {
            if (0 == dev.compare(0, (*it).size(), *it))
            {
              SerialPorts.push_back(sPath + '/' + dev);
              break;
            }
          }
        }
      }
    } while (entry != NULL);

    ulCloseDir(dir);
  }
  
  #endif
#endif
  
  return SerialPorts.size();
}


