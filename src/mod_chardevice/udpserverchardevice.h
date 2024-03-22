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
// udpserverchardevice.h
//
//    Cross-platform interface to a single-connection UDP server.
//
// authors
//
//    Todd R. Templeton <ttemplet@eecs.berkeley.edu>
//
//----------------------------------------------------------------------------------


#ifndef __UDPSERVERCHARDEVICE_H__
#define __UDPSERVERCHARDEVICE_H__

#include "socketchardevicebase.h"


// character device for UDP server communication
class UdpServerCharDevice : public SocketCharDevice
{
private:

    // address of connected client
    sockaddr_in connectedClientAddr;
    // whether address of connected client is valid
    bool connectedClientAddrValid;
    
    
protected:

    // close connection (returns 0 on success or -1 on error)
    int close_connection( void );

    
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
    // close current client (returns 0 on success or -1 on error)
    int close_client( void );
    // whether currently connected
    bool connected( void );
    
    
    // constructor (_options = options string, _wait = whether to wait for connection/reconnection)
    UdpServerCharDevice( const char *_options, bool _wait = true ) : SocketCharDevice( _options, _wait )
    {
        int __attribute__((unused)) retval;
    
        connectedClientAddrValid = false;
        retval = this->open( );
        assert( !wait || retval != -1 );
    }
    
    
    // destructor
    ~UdpServerCharDevice( )
    {
        this->close( );
    }
};


#endif // __UDPSERVERCHARDEVICE_H__
