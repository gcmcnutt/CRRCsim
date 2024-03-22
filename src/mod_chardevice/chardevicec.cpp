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
// chardevicec.cpp
//
//    C interface to chardevice, a C-ified version of CharDeviceWrapper.
//
// authors
//
//    Todd R. Templeton <ttemplet@eecs.berkeley.edu>
//
//----------------------------------------------------------------------------------


#include <stdlib.h>
#include <assert.h>
#include "chardevice.h"
#include "chardevicec.h"


struct CharDeviceCInt
{
    CharDeviceWrapper *charDevice;
};


// constructor (options = options string, waitForConnection = whether to wait for connection/reconnection, readFromLog = whether to read from the read log (instead of normal operation))
//FIXME: logging options
CharDeviceC char_device_new( const char *options, int waitForConnection )
{
    CharDeviceC d = new CharDeviceCInt;
    d->charDevice = new CharDeviceWrapper( options, waitForConnection ? true : false );
    return d;
}


// destructor
void char_device_free( CharDeviceC *d )
{
    assert( d && *d && (*d)->charDevice );
    delete (*d)->charDevice;
    delete *d;
    *d = 0;
}


// read from device (buf = buffer into which to read, count = max number of bytes to read,
//   uses maxInterval and wait, returns number of bytes read or -1 on error)
ssize_t char_device_read( CharDeviceC d, void *buf, size_t count )
{
    assert( d && d->charDevice );
    return d->charDevice->read( buf, count );
}


// write to device (buf = buffer from which to write, count = number number of bytes to write,
//   uses wait, returns number of bytes written or -1 on error)
ssize_t char_device_write( CharDeviceC d, void *buf, size_t count )
{
    assert( d && d->charDevice );
    return d->charDevice->write( buf, count );
}


// set whether to wait for connection/reconnection
void char_device_set_wait_for_connection( CharDeviceC d, int waitForConnection )
{
    assert( d && d->charDevice );
    d->charDevice->set_wait_for_connection( waitForConnection ? true : false );
}


// set time in seconds since last successful read after which to determine that
//   link has gone down
void char_device_set_max_read_interval( CharDeviceC d, double maxInterval )
{
    assert( d && d->charDevice );
    d->charDevice->set_max_read_interval( maxInterval );
}


// set whether to wait for at least one byte before returning from a read
void char_device_set_wait_for_data( CharDeviceC d, int waitForData )
{
    assert( d && d->charDevice );
    d->charDevice->set_wait_for_data( waitForData ? true : false );
}


// enable logging
void char_device_enable_logging( CharDeviceC d )
{
    assert( d && d->charDevice );
    d->charDevice->enable_logging( );
}


// disable logging
void char_device_disable_logging( CharDeviceC d )
{
    assert( d && d->charDevice );
    d->charDevice->disable_logging( );
}


// return whether logging is enabled
int char_device_is_logging_enabled( CharDeviceC d )
{
    assert( d && d->charDevice );
    return d->charDevice->is_logging_enabled( ) ? 1 : 0;
}


// run script from file (returns -1 on error)
int char_device_run_script( CharDeviceC d, const char *scriptfile )
{
    assert( d && d->charDevice );
    return d->charDevice->run_script( scriptfile );
}


// set function call for getting the current time (in microseconds)
void char_device_set_time_function( CharDeviceC d, uint64_t (*get_time)( void ) )
{
    assert( d && d->charDevice );
    d->charDevice->set_time_function( get_time );
}
