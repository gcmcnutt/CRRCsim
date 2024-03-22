//----------------------------------------------------------------------------------//
//                                                                                  //
//  Created on November 09, 2006                                                    //
//                                                                                  //
//  "Copyright (c) 2006 The Regents of the University of California.                //
//  All rights reserved.                                                            //
//                                                                                  //
//  Permission to use, copy, modify, and distribute this software and its           //
//  documentation for any purpose, without fee, and without written agreement is    //
//  hereby granted, provided that the above copyright notice, the following         //
//  two paragraphs and the author appear in all copies of this software.            //
//                                                                                  //
//  IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO ANY PARTY FOR       //
//  DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT     //
//  OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF    //
//  CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                  //
//                                                                                  //
//  THE UNIVERSITY OF CALIFORNIA SPECIFICALLY DISCLAIMS ANY WARRANTIES,             //
//  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY        //
//  AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS       //
//  ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION TO      //
//  PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS."         //
//                                                                                  //
//----------------------------------------------------------------------------------//


//----------------------------------------------------------------------------------
//
// serialchardevice.cpp
//
//    Cross-platform interface to serial ports.
//
// authors
//
//    Todd R. Templeton <ttemplet@eecs.berkeley.edu>
//    David H. Shim <hcshim@eecs.berkeley.edu> (QNX serial initialization)
//
//----------------------------------------------------------------------------------


#include "serialchardevice.h"
#include "chardevicecommon.h"
#ifdef NBMINGW
#include <pins.h> // for enabling rs485
#include <pinconstant.h> // for enabling rs485
#endif


#ifdef WIN32

// open device, called first time by constructor (uses wait, returns -1 on error)
// for info about win32 serial API, see http://www.robbayer.com/files/serial-win.pdf
int SerialCharDevice::open(void)
{
    int i;
    int len = strlen( options ), portlen;
    char *szport, *szport2 = 0;
    int baudrate;
    DCB params = {0};
    int baudmask;
    
    // find serial port device
    for( i = 0; i < len && options[ i ] != ','; i++ );
    assert( ( i + 1 ) < len );
    options[ i ] = '\0';
    szport = options;
    // find baud rate
    baudrate = atoi( &options[ i + 1 ] );

    // fix serial port device, e.g. COM1 or COM1: -> \\.\COM1
    portlen = strlen( szport );
    if( portlen > 3 && ( szport[ 0 ] == 'C' || szport[ 0 ] == 'c' ) && ( szport[ 1 ] == 'O' || szport[ 1 ] == 'o' ) && ( szport[ 2 ] == 'M' || szport[ 2 ] == 'm' ) ) {
        int j;
        bool fixport = true;
        if( szport[ portlen - 1 ] == ':' )
            portlen--;
        for( j = 3; j < portlen; j++ ) {
            if( szport[ j ] < '0' || szport[ j ] > '9' ) {
                fixport = false;
                break;
            }
        }
        if( fixport ) {
            szport2 = new char[ portlen + 5 ];
            strcpy( szport2, "\\\\.\\COM" );
            strncpy( &szport2[ 7 ], &szport[ 3 ], portlen - 3 );
            szport2[ portlen + 4 ] = '\0';
            szport = szport2;
        }
    }

    fprintf(stderr, "InitSer: opening %s with baudrate %d...\n", szport, baudrate);
    fd = CreateFile(szport, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0 ); 
    if (fd == INVALID_HANDLE_VALUE) {
        char lastError[1024];
        FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, NULL, GetLastError(), MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), lastError, 1024, NULL);
        fputs(lastError, stderr);
        #if defined(CHARDEVICE_ENABLE_EXCEPTIONS) && CHARDEVICE_ENABLE_EXCEPTIONS != 0
        throw ConfigureDeviceException();
        #else
        exit(-1);
        #endif
    }
    
    switch(baudrate)
    {
    case 110:
        baudmask = CBR_110;
        break;
    case 300:
        baudmask = CBR_300;
        break;
    case 600:
        baudmask = CBR_600;
        break;
    case 1200:
        baudmask = CBR_1200;
        break;
    case 2400:
        baudmask = CBR_2400;
        break;
    case 4800:
        baudmask = CBR_4800;
        break;
    case 9600:
        baudmask = CBR_9600;
        break;
    case 19200:
        baudmask = CBR_19200;
        break;
    case 38400:
        baudmask = CBR_38400;
        break;
    case 57600:
        baudmask = CBR_57600;
        break;
    case 115200:
        baudmask = CBR_115200;
        break;
    default:
        fprintf(stderr, "Unknown baudrate %d\r\n", baudrate);
        baudmask = CBR_38400;
    }

    params.DCBlength = sizeof(params);
    if(!GetCommState(fd, &params)) {
        char lastError[1024];
        FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, NULL, GetLastError(), MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), lastError, 1024, NULL);
        fputs(lastError, stderr);
        #if defined(CHARDEVICE_ENABLE_EXCEPTIONS) && CHARDEVICE_ENABLE_EXCEPTIONS != 0
        throw ConfigureDeviceException();
        #else
        exit(-1);
        #endif
    }
    params.BaudRate = baudmask;
    params.ByteSize = 8;
    params.StopBits = ONESTOPBIT;
    params.Parity   = NOPARITY;
    if(!SetCommState(fd, &params)) {
        char lastError[1024];
        FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, NULL, GetLastError(), MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), lastError, 1024, NULL);
        fputs(lastError, stderr);
        #if defined(CHARDEVICE_ENABLE_EXCEPTIONS) && CHARDEVICE_ENABLE_EXCEPTIONS != 0
        throw ConfigureDeviceException();
        #else
        exit(-1);
        #endif
    }
    
    set_wait_for_data(waitForData);
    
    // repair options
    options[ i ] = ',';

    // clean up
    if( szport2 )
        delete[] szport2;
    
    return fd == INVALID_HANDLE_VALUE ? -1 : 0;
}

// set whether to wait for at least one byte before returning from a read
void SerialCharDevice::set_wait_for_data(bool _waitForData)
{
    COMMTIMEOUTS timeouts = {0};
    
    waitForData = _waitForData;
    
    if(!GetCommTimeouts(fd, &timeouts)) {
        char lastError[1024];
        FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, NULL, GetLastError(), MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), lastError, 1024, NULL);
        fputs(lastError, stderr);
        #if defined(CHARDEVICE_ENABLE_EXCEPTIONS) && CHARDEVICE_ENABLE_EXCEPTIONS != 0
        throw ConfigureDeviceException();
        #else
        exit(-1);
        #endif
    }
    if(_waitForData) {
        timeouts.ReadIntervalTimeout        = MAXDWORD;
        timeouts.ReadTotalTimeoutConstant   = MAXDWORD - 1;
        timeouts.ReadTotalTimeoutMultiplier = MAXDWORD;
    }
    else {
        timeouts.ReadIntervalTimeout        = MAXDWORD;
        timeouts.ReadTotalTimeoutConstant   = 0;
        timeouts.ReadTotalTimeoutMultiplier = 0;
    }
    timeouts.WriteTotalTimeoutConstant   = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    if(!SetCommTimeouts(fd, &timeouts)) {
        char lastError[1024];
        FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, NULL, GetLastError(), MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), lastError, 1024, NULL);
        fputs(lastError, stderr);
        #if defined(CHARDEVICE_ENABLE_EXCEPTIONS) && CHARDEVICE_ENABLE_EXCEPTIONS != 0
        throw ConfigureDeviceException();
        #else
        exit(-1);
        #endif
    }
}

 // read values of input (CTS, DSR, RNG, and CAR) pins
void SerialCharDevice::read_input_pins( bool *cts, bool *dsr, bool *rng, bool *car )
{
    unsigned long flags;
    if( GetCommModemStatus( fd, &flags ) == 0 )
    {
        char lastError[1024];
        FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, NULL, GetLastError(), MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), lastError, 1024, NULL);
        fputs(lastError, stderr);
        #if defined(CHARDEVICE_ENABLE_EXCEPTIONS) && CHARDEVICE_ENABLE_EXCEPTIONS != 0
        throw ConfigureDeviceException();
        #else
        exit(-1);
        #endif
    }
    if( cts )
        *cts = flags & MS_CTS_ON ? true : false;
    if( dsr )
        *dsr = flags & MS_DSR_ON ? true : false;
    if( rng )
        *rng = flags & MS_RING_ON ? true : false;
    if( car )
        *car = flags & MS_RLSD_ON ? true : false;
}

// write values of output (RTS and DTR) pins
void SerialCharDevice::write_output_pins( WriteOutputPin rts, WriteOutputPin dtr )
{
    int retval = 1;
    switch( rts )
    {
    case WriteOutputPinLow:
        retval = EscapeCommFunction( fd, CLRRTS );
        break;
    case WriteOutputPinHigh:
        retval = EscapeCommFunction( fd, SETRTS );
        break;
    default:
        break;
    }
    if( retval == 0 )
    {
        char lastError[1024];
        FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, NULL, GetLastError(), MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), lastError, 1024, NULL);
        fputs(lastError, stderr);
        #if defined(CHARDEVICE_ENABLE_EXCEPTIONS) && CHARDEVICE_ENABLE_EXCEPTIONS != 0
        throw ConfigureDeviceException();
        #else
        exit(-1);
        #endif
    }
    switch( dtr )
    {
    case WriteOutputPinLow:
        retval = EscapeCommFunction( fd, CLRDTR );
        break;
    case WriteOutputPinHigh:
        retval = EscapeCommFunction( fd, SETDTR );
        break;
    default:
        break;
    }
    if( retval == 0 )
    {
        char lastError[1024];
        FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, NULL, GetLastError(), MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), lastError, 1024, NULL);
        fputs(lastError, stderr);
        #if defined(CHARDEVICE_ENABLE_EXCEPTIONS) && CHARDEVICE_ENABLE_EXCEPTIONS != 0
        throw ConfigureDeviceException();
        #else
        exit(-1);
        #endif
    }
}

#elif defined(__QNX__)

// open device, called first time by constructor (uses wait, returns -1 on error)
// Serial initialization routine from David Shim
int SerialCharDevice::open(void)
{
    int i;
    int len = strlen( options );
    char *szport;
    int baudrate;
    //int fd;
    struct termios termios_p;
    int baudmask;
    char buf[30];
    
    // find serial port device
    for( i = 0; i < len && options[ i ] != ','; i++ );
    assert( ( i + 1 ) < len );
    options[ i ] = '\0';
    szport = options;
    // find baud rate
    baudrate = atoi( &options[ i + 1 ] );

    fd = open( szport,  O_RDWR); /* open ser*/
    if ( fd <= NULL ) {
        fprintf( stderr, "Error opening device %s for input/output\r\n", szport );
        #if defined(CHARDEVICE_ENABLE_EXCEPTIONS) && CHARDEVICE_ENABLE_EXCEPTIONS != 0
        throw ConfigureDeviceException();
        #else
        exit( -1 );
        #endif
    }
    tcgetattr(fd, &termios_p);         /*load termios_p w/ current params */

    switch(baudrate)
    {
    case 0:
        baudmask = B0;
        break;
    case 50:
        baudmask = B50;
        break;
    case 75:
        baudmask = B75;
        break;
    case 110:
        baudmask = B110;
        break;
    case 134:
        baudmask = B134;
        break;
    case 150:
        baudmask = B150;
        break;
    case 200:
        baudmask = B200;
        break;
    case 300:
        baudmask = B300;
        break;
    case 600:
        baudmask = B600;
        break;
    case 1200:
        baudmask = B1200;
        break;
    case 1800:
        baudmask = B1800;
        break;
    case 2400:
        baudmask = B2400;
        break;
    case 4800:
        baudmask = B4800;
        break;
    case 9600:
        baudmask = B9600;
        break;
    case 19200:
        baudmask = B19200;
        break;
    case 38400:
        baudmask = B38400;
        break;
    case 57600:
        baudmask = B57600;
        break;
    case 115200:
        baudmask = B115200;
        break;
    case 230400:
        baudmask = B230400;
        break;
    default:
        fprintf(stderr, "Unknown baudrate %d\r\n", baudrate);
    }
    
    cfsetispeed(&termios_p, baudmask);     /* set baud rates*/
    cfsetospeed(&termios_p, baudmask);
    termios_p.c_cflag = CS8|CREAD;//|PARENB|PARODD; // no flow control |IHFLOW|OHFLOW;
    tcsetattr(fd, TCSANOW, &termios_p);     
    dev_mode(fd, 0, _DEV_MODES);
    dev_state(fd, 0, _DEV_EVENT_INPUT);
    tcflush(fd,TCIOFLUSH);
    while (dev_ischars(fd))
        dev_read(fd,buf,sizeof(buf),sizeof(buf),0,0,0,0);
    
    set_wait_for_data(waitForData);
        
    // repair options
    options[ i ] = ',';

    return(fd);

}

// set whether to wait for at least one byte before returning from a read
void SerialCharDevice::set_wait_for_data( bool _waitForData )
{
    waitForData = _waitForData;
    
    //FIXME: implement this
}

 // read values of input (CTS, DSR, RNG, and CAR) pins
void SerialCharDevice::read_input_pins( bool *cts, bool *dsr, bool *rng, bool *car )
{
    //FIXME: implement this
    assert( 0 );
}

// write values of output (RTS and DTR) pins
void SerialCharDevice::write_output_pins( WriteOutputPin rts, WriteOutputPin dtr )
{
    //FIXME: implement this
    assert( 0 );
}

#elif defined(NBMINGW)

// open device, called first time by constructor (uses wait, returns -1 on error)
int SerialCharDevice::open( void )
{
    int i;
    int len = strlen( options );
    int baudrate;
    //int fd;
    
    // find serial port device
    for( i = 0; i < len && options[ i ] != ','; i++ );
    assert( ( i + 1 ) < len );
    options[ i ] = '\0';
    portnum = atoi( options );
    // find baud rate
    baudrate = atoi( &options[ i + 1 ] );

    fprintf( stderr, "InitSer: opening %d with baudrate %d...\n", portnum, baudrate );
    fd = OpenSerial( portnum, baudrate, 1, 8, eParityNone );
    if( fd < 0 )
    {
        fprintf( stderr, "Error opening device %d for input/output\n", portnum );
        #if defined(CHARDEVICE_ENABLE_EXCEPTIONS) && CHARDEVICE_ENABLE_EXCEPTIONS != 0
        throw ConfigureDeviceException();
        #else
        exit( -1 );
        #endif
    }    
    
    set_wait_for_data( waitForData );
    
    // enable RS-485 output for UART0
    if( portnum == 0 )
    {
        J2[38] = 0;
        J2[38].function( PINJ2_38_GPIO );
    }
    
    // repair options
    options[ i ] = ',';
    
    return fd;
}

// set whether to wait for at least one byte before returning from a read
void SerialCharDevice::set_wait_for_data( bool _waitForData )
{
    //NOTE: cannot set port as nonblocking on NetBurner, but can check if data is available to read
    waitForData = _waitForData;
}

// read values of input (CTS, DSR, RNG, and CAR) pins
void SerialCharDevice::read_input_pins( bool *cts, bool *dsr, bool *rng, bool *car )
{
    if( cts )
    {
        switch( portnum )
        {
        case 0:
            J2[29].function( PINJ2_29_GPIO );
            *cts = J2[29] ? true : false;
            break;
        case 1:
            J2[24].function( PINJ2_24_GPIO );
            *cts = J2[24] ? true : false;
            break;
        default:
            *cts = false;
            break;
        }
    }
    if( dsr )
        *dsr = false;
    if( rng )
        *rng = false;
    if( car )
        *car = false;
}

// write values of output (RTS and DTR) pins
void SerialCharDevice::write_output_pins( WriteOutputPin rts, WriteOutputPin dtr )
{
    if( rts != WriteOutputPinUnchanged )
    {
        switch( portnum )
        {
        case 0:
            // we do not want to set J2[38] because it is being used to enable the transmitter
            break;
        case 1:
            J2[23] = rts;
            J2[23].function( PINJ2_23_GPIO );
            break;
        default:
            break;
        }
    }
}

#else

// open device, called first time by constructor (uses wait, returns -1 on error)
int SerialCharDevice::open(void)
{
    int i;
    int len = strlen( options );
    char *szport;
    int baudrate;
    //int fd;
    struct termios oldtio,newtio;
    int baudmask;
    
    // find serial port device
    for( i = 0; i < len && options[ i ] != ','; i++ );
    assert( ( i + 1 ) < len );
    options[ i ] = '\0';
    szport = options;
    // find baud rate
    baudrate = atoi( &options[ i + 1 ] );

    fprintf(stderr, "InitSer: opening %s with baudrate %d...\n", szport, baudrate);
    fd = ::open(szport, O_RDWR | O_NOCTTY ); 
    if (fd <0)
    {
        perror(szport);
        #if defined(CHARDEVICE_ENABLE_EXCEPTIONS) && CHARDEVICE_ENABLE_EXCEPTIONS != 0
        throw ConfigureDeviceException();
        #else
        exit(-1);
        #endif
    }

    tcgetattr(fd,&oldtio); /* save current serial port settings */
    bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

    newtio.c_cflag = /*baudrate  |*/ CS8 | CLOCAL | CREAD /*| CRTSCTS*/;
    newtio.c_iflag = IGNPAR; // | ICRNL; ->IGNORE CARRIGE RETURN, WHICH IS A BIG NO-NO FOR BINARY STREAM
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;//ICANON; // DISABLED ANYTHING SUSPICIOUS AND SEEM UNNECESSARY
    newtio.c_cc[VINTR]    = 0;     /* Ctrl-c */ 
    newtio.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
    newtio.c_cc[VERASE]   = 0;     /* del */
    newtio.c_cc[VKILL]    = 0;     /* @ */
    newtio.c_cc[VEOF]     = 0;     /* Ctrl-d */
    newtio.c_cc[VTIME]    = 0;     /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 0;     /* blocking read until 0 characters arrive (non-blocking) */
    #ifdef VSWTC
    /* ensured above that either both or neither are defined */
    newtio.c_cc[VSWTC]    = 0;     /* '\0' */
    newtio.c_cc[VSWTCH]   = 0;     /* '\0' */
    #endif
    newtio.c_cc[VSTART]   = 0;     /* Ctrl-q */ 
    newtio.c_cc[VSTOP]    = 0;     /* Ctrl-s */
    newtio.c_cc[VSUSP]    = 0;     /* Ctrl-z */
    newtio.c_cc[VEOL]     = 0;     /* '\0' */
    newtio.c_cc[VREPRINT] = 0;     /* Ctrl-r */
    newtio.c_cc[VDISCARD] = 0;     /* Ctrl-u */
    newtio.c_cc[VWERASE]  = 0;     /* Ctrl-w */
    newtio.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
    newtio.c_cc[VEOL2]    = 0;     /* '\0' */
    
    switch(baudrate)
    {
    case 0:
        baudmask = B0;
        break;
    case 50:
        baudmask = B50;
        break;
    case 75:
        baudmask = B75;
        break;
    case 110:
        baudmask = B110;
        break;
    case 134:
        baudmask = B134;
        break;
    case 150:
        baudmask = B150;
        break;
    case 200:
        baudmask = B200;
        break;
    case 300:
        baudmask = B300;
        break;
    case 600:
        baudmask = B600;
        break;
    case 1200:
        baudmask = B1200;
        break;
    case 1800:
        baudmask = B1800;
        break;
    case 2400:
        baudmask = B2400;
        break;
    case 4800:
        baudmask = B4800;
        break;
    case 9600:
        baudmask = B9600;
        break;
    case 19200:
        baudmask = B19200;
        break;
    case 38400:
        baudmask = B38400;
        break;
    case 57600:
        baudmask = B57600;
        break;
    case 115200:
        baudmask = B115200;
        break;
    case 230400:
        baudmask = B230400;
        break;
    default:
        fprintf(stderr, "Unknown baudrate %d\r\n", baudrate);
        baudmask = B38400;
    }
    
    cfsetispeed(&newtio, baudmask);
    cfsetospeed(&newtio, baudmask);
    
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd,TCSANOW,&newtio);
    
    set_wait_for_data(waitForData);
    
    // repair options
    options[ i ] = ',';
    
    return fd;
}

// set whether to wait for at least one byte before returning from a read
void SerialCharDevice::set_wait_for_data( bool _waitForData )
{
    struct termios newtio;
    
    waitForData = _waitForData;
    
    tcgetattr( fd, &newtio );
    if( _waitForData )
        newtio.c_cc[VMIN] = 1;     /* blocking read until 1 character arrives */    
    else
        newtio.c_cc[VMIN] = 0;     /* blocking read until 0 characters arrive (non-blocking) */
    tcsetattr( fd, TCSANOW, &newtio );
}

 // read values of input (CTS, DSR, RNG, and CAR) pins
void SerialCharDevice::read_input_pins( bool *cts, bool *dsr, bool *rng, bool *car )
{
    int flags;
    if( ioctl( fd, TIOCMGET, &flags ) != 0 )
    {
        #if defined(CHARDEVICE_ENABLE_EXCEPTIONS) && CHARDEVICE_ENABLE_EXCEPTIONS != 0
        throw ConfigureDeviceException();
        #else
        exit(-1);
        #endif
    }
    if( cts )
        *cts = flags & TIOCM_CTS ? true : false;
    if( dsr )
        *dsr = flags & TIOCM_DSR ? true : false;
    if( rng )
        *rng = flags & TIOCM_RNG ? true : false;
    if( car )
        *car = flags & TIOCM_CAR ? true : false;
}

// write values of output (RTS and DTR) pins
void SerialCharDevice::write_output_pins( WriteOutputPin rts, WriteOutputPin dtr )
{
    int flags;
    if( ioctl( fd, TIOCMGET, &flags ) != 0 )
    {
        #if defined(CHARDEVICE_ENABLE_EXCEPTIONS) && CHARDEVICE_ENABLE_EXCEPTIONS != 0
        throw ConfigureDeviceException();
        #else
        exit(-1);
        #endif
    }
    switch( rts )
    {
    case WriteOutputPinLow:
        flags &= ~TIOCM_RTS;
        break;
    case WriteOutputPinHigh:
        flags |= TIOCM_RTS;
        break;
    default:
        break;
    }
    switch( dtr )
    {
    case WriteOutputPinLow:
        flags &= ~TIOCM_DTR;
        break;
    case WriteOutputPinHigh:
        flags |= TIOCM_DTR;
        break;
    default:
        break;
    }
    if( ioctl( fd, TIOCMSET, &flags ) != 0 )
    {
        #if defined(CHARDEVICE_ENABLE_EXCEPTIONS) && CHARDEVICE_ENABLE_EXCEPTIONS != 0
        throw ConfigureDeviceException();
        #else
        exit(-1);
        #endif
    }
}

#endif

// read from device (buf = buffer into which to read, count = max number of bytes to read,
//   uses maxInterval and wait, returns number of bytes read or -1 on error)
ssize_t SerialCharDevice::read( void *buf, size_t count )
{
    ssize_t retval = 0;
    //fprintf( stderr, "Reading up to %d bytes\n", count );
    do
    {
        #ifdef WIN32
        DWORD _retval;
        if( !ReadFile( fd, buf, count, &_retval, NULL ) )
            retval = -1;
        else
            retval = _retval;
        #else
        #ifdef NBMINGW
        if( !dataavail( fd ) && !waitForData )
        {
            retval = 0;
            break;
        }
        #endif
        retval = ::read( fd, (read_write_type)buf, count );
        #endif
    } while( retval == 0 && waitForData );
    return retval;
}

// write to device (buf = buffer from which to write, count = number number of bytes to write,
//   uses wait, returns number of bytes written or -1 on error)
ssize_t SerialCharDevice::write( const void *buf, size_t count )
{
    //fprintf( stderr, "Writing %d bytes\n", count );
    #ifdef WIN32
    DWORD retval;
    if( !WriteFile( fd, buf, count, &retval, NULL ) )
        return -1;
    return retval;
    #else
    return ::write( fd, (const read_write_type)buf, count );
    #endif
}

// close device, called last time by destructor (returns 0 on success or -1 on error)
int SerialCharDevice::close( void )
{
    int retval = 0;
    if( fd != INVALID_HANDLE_VALUE )
    {
        #ifdef WIN32
        retval = CloseHandle( fd );
        retval = ( retval == 0 ) ? -1 : 0;
        #elif defined(NBMINGW)
        retval = SerialClose( portnum );
        retval = ( retval == 0 ) ? -1 : 0;
        #else
        retval = ::close( fd );
        #endif
        fd = INVALID_HANDLE_VALUE;
    }
    return retval;
}
