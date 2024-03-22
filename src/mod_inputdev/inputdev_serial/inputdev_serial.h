/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2008 - Martin Herrmann (original author)
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
#ifndef T_TX_INTERFACE_SERIAL
#define T_TX_INTERFACE_SERIAL

#include "../inputdev.h"
#include "../../mod_misc/SimpleXMLTransfer.h"
#include "../../mod_chardevice/serialchardevice.h"
#include <string>

using namespace std;

//#define READ_BUFFER_SIZE  (256)   /* max. line length for config files  */
//#define SYNC_TIMEOUT      (500)   /* try to sync for approx. 5 seconds  */

/** \brief An interface using the serial port
 *
 * This is an abstract base class for interfaces which use a transmitter
 * connected to a serial port.
 *
 * There are multiple interfaces using the serial port, which leads to code
 * duplication. What's worse, some of them might not implement configuration or
 * support for a particular OS.
 *
 * So this base class can be used for all interfaces using the serial port. It
 * was written for the Zhen Hua interface. I suggest modifying all serial input
 * classes to inherit from this  class.
 *
 * Implementations of this class need to:
 *   - implement processDataByte and call setRawData from there
 *   - implement getInterfaceName (needed for storing settings)
 *   - override inputMethod to return the appropriate constant from T_TX_Interface::eIM_...
 * Implementations should:
 *   - override getNumAxes
 * Implementations may:
 *   - override the init method to do things like setting DTR and RTS after
 *     calling the original
 *
 * This class uses SerialCharDevice.
 */
class T_TX_InterfaceSerial: public T_TX_Interface
{
  public:
    // Construction
    T_TX_InterfaceSerial ();
    virtual ~T_TX_InterfaceSerial ();

    // Configuration
    virtual int init (SimpleXMLTransfer *config); // Initialize the interface, read configuration data
    virtual void putBackIntoCfg (SimpleXMLTransfer *config); // Write configuration back

    virtual std::string getDeviceName ();
    virtual void setDeviceName (std::string devname); // ttyS0, ttyUSB0, COM1...

    virtual int getDeviceSpeed ();
    virtual void setDeviceSpeed (int speed);

    // Data access
    virtual void getInputData (TSimInputs *inputs); // Called by the simulator to get the input data
    virtual void getRawData(float* target); // Needed for calibration

    // I/O
    void readSerialData ();
    virtual void setRawData (int num, float data);
    static const char *baudRates[];

    // Interface
    virtual string getErrMsg ()
    {
      return errorMessage;
    };
    
    // Return a list of possible interfaces
    static int getDeviceList(std::vector<std::string>& SerialPorts);

  protected:
    virtual std::string getInterfaceName ()=0; // Implementations need to implement this
    virtual std::string getXmlChildName ()
    {
      return "inputMethod."+getInterfaceName ();
    }
    virtual int getDefaultBaudRate ()
    {
      return 19200;
    }
    virtual int getBaudRate() const
    {
      return baudRate;
    }
    virtual void processDataByte (unsigned char byte)=0; // Implementations need to implement this
    virtual void setErrMsg (string msg)
    {
      errorMessage=msg;
    }
    virtual int baudRateToSpeed (int baudRate);

    virtual void setDtr (bool dtr);
    virtual void setRts (bool rts);



  private:
    int getSerialByte ();
    SerialCharDevice *charDevice;
    string portName;
    unsigned int baudRate;

    static const int nNumBaudrates;
    static const int anBaudRates[];

    float rawData[TX_MAXAXIS];

    string errorMessage;

#if DEBUG_TX_INTERFACE>0
#define DBGBUFFERSIZE (256)
    unsigned char dbgbuffer[DBGBUFFERSIZE];
    int dbgbufferidx;
    void debugHandler(unsigned char ucByte);
#endif
};

#endif // TX_INTERFACE_SERIAL

