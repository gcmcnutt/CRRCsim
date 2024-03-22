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
// serialchardevice.h
//
//    Cross-platform interface to serial ports.
//
// authors
//
//    Todd R. Templeton <ttemplet@eecs.berkeley.edu>
//    David H. Shim <hcshim@eecs.berkeley.edu> (QNX serial initialization)
//
//----------------------------------------------------------------------------------


#ifndef __SERIALCHARDEVICE_H__
#define __SERIALCHARDEVICE_H__

#include "chardevicebase.h"


// character device for serial communication
class SerialCharDevice : public CharDevice
{
private:

    // private copy of options string
    char *options;
    // device handle
    HANDLE fd;
    #ifdef NBMINGW
    // serial port number
    int portnum;
    #endif

public:

    // open device, called first time by constructor (uses wait, returns -1 on error)
    int open( void );
    // read from device (buf = buffer into which to read, count = max number of bytes to read,
    //   uses maxInterval and wait, returns number of bytes read or -1 on error)
    ssize_t read( void *buf, size_t count );
    // write to device (buf = buffer from which to write, count = number number of bytes to write,
    //   uses wait, returns number of bytes written or -1 on error)
    ssize_t write( const void *buf, size_t count );
    // close device, called last time by destructor (returns 0 on success or -1 on error)
    int close( void );
    
    
    // set whether to wait for at least one byte before returning from a read
    void set_wait_for_data( bool _waitForData );

    
    enum WriteOutputPin
    {
        WriteOutputPinLow = 0,
        WriteOutputPinHigh = 1,
        WriteOutputPinUnchanged = 2
    };
    // read values of input (CTS, DSR, RNG, and CAR) pins
    void read_input_pins( bool *cts, bool *dsr, bool *rng, bool *car );
    // write values of output (RTS and DTR) pins
    void write_output_pins( WriteOutputPin rts, WriteOutputPin dtr );
    
    
    // constructor (_options = options string, _wait = whether to wait for connection/reconnection)
    SerialCharDevice( const char *_options, bool _wait = true )
    {
        int __attribute__((unused)) retval;
    
        this->set_wait_for_connection( _wait );
        options = new char[ strlen( _options ) + 1 ];
        strcpy( options, _options );
        fd = INVALID_HANDLE_VALUE;
        retval = this->open( );
        assert( retval != -1 );
    }
    
    
    // destructor
    ~SerialCharDevice( )
    {
        this->close( );
        delete[] options;
    }
};


#endif // __SERIALCHARDEVICE_H__
