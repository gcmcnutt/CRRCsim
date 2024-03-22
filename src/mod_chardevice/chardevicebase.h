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
// chardevicebase.h
//
//    Base functionality that is used by many chardevices.
//
// authors
//
//    Todd R. Templeton <ttemplet@eecs.berkeley.edu>
//
//----------------------------------------------------------------------------------


#ifndef __CHARDEVICEBASE_H__
#define __CHARDEVICEBASE_H__


#if (defined(WIN32) && !defined(__GNUC__)) || defined(__EXCEPTIONS)
#define CHARDEVICE_ENABLE_EXCEPTIONS 1
#endif


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#if defined(CHARDEVICE_ENABLE_EXCEPTIONS) && CHARDEVICE_ENABLE_EXCEPTIONS != 0
#include <stdexcept>
#endif
#ifdef WIN32
#include <winsock.h>
#elif defined(NBMINGW)
#include <basictypes.h>
#include <sys/types.h>
#else // ifdef WIN32
#include <unistd.h> // usleep
#include <netinet/in.h>
#endif // ifdef WIN32


#ifndef WIN32
#define INVALID_HANDLE_VALUE -1
#define INVALID_SOCKET -1
typedef int HANDLE;
typedef int SOCKET;
#endif

#if defined(WIN32) && !defined(__GNUC__)
typedef int ssize_t;
typedef __int8 int8_t;
typedef unsigned __int8 uint8_t;
typedef __int16 int16_t;
typedef unsigned __int16 uint16_t;
typedef __int32 int32_t;
typedef unsigned __int32 uint32_t;
typedef __int64 int64_t;
//NOTE: to solve errors in Visual Studio 6 about conversion from unsigned __int64 to double not being implemented, install SP5 and the processor pack for Visual Studio 6
typedef unsigned __int64 uint64_t;
#elif defined(NBMINGW)
typedef __int8_t int8_t;
typedef __uint8_t uint8_t;
typedef __int16_t int16_t;
typedef __uint16_t uint16_t;
//typedef __int32_t int32_t;
//typedef __uint32_t uint32_t;
typedef __int64_t int64_t;
typedef __uint64_t uint64_t;
#endif


//typedef unsigned char  BYTE;
//typedef unsigned short int WORD;
//typedef unsigned int DOUBLEWORD;


#define TIME2DOUBLE(t) ((double)t / 1000000.0)

#define MIN(x,y) ((x) < (y) ? (x) : (y))
#define MAX(x,y) ((x) < (y) ? (y) : (x))

#if defined(WIN32) || defined(NBMINGW)
void usleep( unsigned long usec );
#endif

#ifdef NBMINGW
uint16_t htons( uint16_t a );
uint16_t ntohs( uint16_t a );
uint32_t htonl( uint32_t a );
uint32_t ntohl( uint32_t a );
#endif
uint64_t htonll( uint64_t a );
uint64_t ntohll( uint64_t a );

// get elapsed time, first call returns 0
uint64_t get_elapsed_time( void );


// base character device class
class CharDevice
{
public:

    #if defined(CHARDEVICE_ENABLE_EXCEPTIONS) && CHARDEVICE_ENABLE_EXCEPTIONS != 0
    class ConfigureDeviceException : public std::runtime_error
    {
    public:
        ConfigureDeviceException( ) 
          : std::runtime_error( "Error configuring device" ) { }
        
        ConfigureDeviceException(std::string what) 
          : std::runtime_error( std::string("Error configuring device: ") + what) { }
    };
    #endif

protected:

    // whether to wait for connection/reconnection
    bool wait;
    // time in seconds since last successful read after which to determine that
    //   link has gone down
    double maxInterval;
    // whether to wait for at least one byte before returning from a read
    bool waitForData;
    // function call for getting the current time (in microseconds)
    uint64_t (*get_time)( void );

public:

    // open device, called first time by constructor (uses wait, returns -1 on error)
    virtual int open( void ) = 0;
    // read from device (buf = buffer into which to read, count = max number of bytes to read,
    //   uses maxInterval and wait, returns number of bytes read or -1 on error)
    virtual ssize_t read( void *buf, size_t count ) = 0;
    // write to device (buf = buffer from which to write, count = number number of bytes to write,
    //   uses wait, returns number of bytes written or -1 on error)
    virtual ssize_t write( const void *buf, size_t count ) = 0;
    // close device, called last time by destructor (returns 0 on success or -1 on error)
    virtual int close( void ) = 0;
    
    
    // set whether to wait for connection/reconnection
    virtual void set_wait_for_connection( bool _wait )
    {
        wait = _wait;
    }
    // set time in seconds since last successful read after which to determine that
    //   link has gone down
    virtual void set_max_read_interval( double _maxInterval )
    {
        maxInterval = _maxInterval;
    }
    // set whether to wait for at least one byte before returning from a read
    virtual void set_wait_for_data( bool _waitForData )
    {
        waitForData = _waitForData;
    }
    
    
    // set function call for getting the current time (in microseconds)
    virtual void set_time_function( uint64_t (*_get_time)( void ) )
    {
        get_time = _get_time;
    }
    // set default function call for getting the current time (in microseconds)
    static void set_default_time_function( uint64_t (*_get_time)( void ) );
    
    
    // constructor
    CharDevice( );
    
    
    // destructor
    virtual ~CharDevice( )
    {
    }
};


#endif // __CHARDEVICEBASE_H__
