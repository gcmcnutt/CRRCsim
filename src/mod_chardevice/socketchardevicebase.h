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
// socketchardevicebase.h
//
//    Base class for cross-platform interfaces to sockets (TCP and UDP).
//
// authors
//
//    Todd R. Templeton <ttemplet@eecs.berkeley.edu>
//
//----------------------------------------------------------------------------------


#ifndef __SOCKETCHARDEVICEBASE_H__
#define __SOCKETCHARDEVICEBASE_H__

#include "chardevicebase.h"
#ifdef NBMINGW
#include <nettypes.h>
#endif


#ifdef WIN32
typedef SOCKADDR_IN sockaddr_in;
#elif defined(NBMINGW)
struct sockaddr_in
{
    struct
    {
        IPADDR s_addr;
    } sin_addr;
    uint16_t sin_port;
};
#endif


class SocketCharDevice : public CharDevice
{
protected:

    // private copy of options string
    char *options;
    // device handle
    SOCKET fd;
    // time of last successful read
    double lastReadTime;

    
private:
    
    // initialize socket library, e.g. Windows WGA
    static void init_socket_library( void );
    // cleanup socket library, e.g. Windows WGA
    static void cleanup_socket_library( void );

    
public:
    
    // set whether to wait for at least one byte before returning from a read
    void set_wait_for_data( bool _waitForData );

    
protected:

    // close connection (returns 0 on success or -1 on error)
    virtual int close_connection( void ) = 0;
    
    
    // set socket as blocking or nonblocking
    static void set_socket_blocking( SOCKET s, bool blocking );
    // set socket to be able to reuse a local address
    static void set_socket_server_reuse_address( SOCKET s );
    // set socket to not use the Nagle algorithm
    static void set_socket_tcp_no_delay( SOCKET s );
    // close a socket (returns 0 on success or -1 on error)
    static int close_socket( SOCKET &s );
    #ifdef NBMINGW
    // determine how many bytes are in next message, and from where it was sent
    static ssize_t peek_socket_udp( SOCKET s, sockaddr_in *addr );
    #endif
    // fill port and host in sockaddr_in structure
    static void fill_sockaddr_in( sockaddr_in &addr, const char *host, int port );
    // test return value from read/recv; returns whether to continue reading
    bool test_recv_retval( ssize_t& retval, ssize_t spaceRemainingInBuffer, bool messageProtocol = false, bool allowPartialMessageRead = false );
    // test return value from write/send; returns whether to continue writing
    bool test_send_retval( ssize_t& retval );


    // constructor (_options = options string, _wait = whether to wait for connection/reconnection)
    SocketCharDevice( const char *_options, bool _wait = true )
    {
        this->set_wait_for_connection( _wait );
        options = new char[ strlen( _options ) + 1 ];
        strcpy( options, _options );
        fd = INVALID_SOCKET;
        lastReadTime = this->get_time( );
        this->init_socket_library( );
    }
    
    
    // destructor
    ~SocketCharDevice( )
    {
        this->cleanup_socket_library( );
        delete[] options;
    }

};

#endif // __SOCKETCHARDEVICEBASE_H__
