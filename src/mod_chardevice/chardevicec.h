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
// chardevicec.h
//
//    C interface to chardevice, a C-ified version of CharDeviceWrapper.
//
// authors
//
//    Todd R. Templeton <ttemplet@eecs.berkeley.edu>
//
//----------------------------------------------------------------------------------


#ifndef __CHARDEVICEC_H__
#define __CHARDEVICEC_H__


#ifdef __cplusplus
extern "C"
{
#endif

//FIXME: this is copied from chardevicebase.h
//#include <stdlib.h>
//#include <stdio.h>
//#include <string.h>
//#include <assert.h>
//#if defined(CHARDEVICE_ENABLE_EXCEPTIONS) && CHARDEVICE_ENABLE_EXCEPTIONS != 0
//#include <stdexcept>
//#endif
#ifdef WIN32
//#include <winsock.h>
#elif defined(NBMINGW)
#include <basictypes.h>
#include <sys/types.h>
#else // ifdef WIN32
//#include <unistd.h> // usleep
#include <netinet/in.h>
#endif // ifdef WIN32

//FIXME: this is copied from chardevicebase.h
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

struct CharDeviceCInt;
typedef struct CharDeviceCInt *CharDeviceC;

// constructor (options = options string, waitForConnection = whether to wait for connection/reconnection, readFromLog = whether to read from the read log (instead of normal operation))
//FIXME: logging options
CharDeviceC char_device_new( const char *options, int waitForConnection );
// destructor
void char_device_free( CharDeviceC *d );

// read from device (buf = buffer into which to read, count = max number of bytes to read,
//   uses maxInterval and wait, returns number of bytes read or -1 on error)
ssize_t char_device_read( CharDeviceC d, void *buf, size_t count );
// write to device (buf = buffer from which to write, count = number number of bytes to write,
//   uses wait, returns number of bytes written or -1 on error)
ssize_t char_device_write( CharDeviceC d, void *buf, size_t count );

// set whether to wait for connection/reconnection
void char_device_set_wait_for_connection( CharDeviceC d, int waitForConnection );

// set time in seconds since last successful read after which to determine that
//   link has gone down
void char_device_set_max_read_interval( CharDeviceC d, double maxInterval );
// set whether to wait for at least one byte before returning from a read
void char_device_set_wait_for_data( CharDeviceC d, int waitForData );

// enable logging
void char_device_enable_logging( CharDeviceC d );
// disable logging
void char_device_disable_logging( CharDeviceC d );
// return whether logging is enabled
int char_device_is_logging_enabled( CharDeviceC d );

// run script from file (returns -1 on error)
int char_device_run_script( CharDeviceC d, const char *scriptfile );

// set function call for getting the current time (in microseconds)
void char_device_set_time_function( CharDeviceC d, uint64_t (*get_time)( void ) );

#ifdef __cplusplus
} // extern "C"
#endif


#endif // __CHARDEVICEC_H__
