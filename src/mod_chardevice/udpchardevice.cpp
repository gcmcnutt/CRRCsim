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
// udpchardevice.cpp
//
//    Cross-platform interface to a UDP client.
//
// authors
//
//    Todd R. Templeton <ttemplet@eecs.berkeley.edu>
//
//----------------------------------------------------------------------------------


#include "udpchardevice.h"
#include "chardevicecommon.h"


// open device, called first time by constructor (uses wait, returns -1 on error)
int UdpCharDevice::open( void )
{
    int i/*, j*/;
    int len = strlen( options );
    char *host;
    int port;
    sockaddr_in addr;
    #ifndef NBMINGW
    int __attribute__((unused)) retval;
    #endif
    
    // find host
    for( i = 0; i < len && options[ i ] != ','; i++ );
    assert( ( i + 1 ) < len );
    options[ i ] = '\0';
    host = options;
    // find port
    port = atoi( &options[ i + 1 ] );
    
    if( fd == INVALID_SOCKET )
    {
        fprintf( stderr, "connecting to %s port %d\n", host, port );

        this->fill_sockaddr_in( addr, host, port );
        #ifdef NBMINGW
        fd = CreateRxTxUdpSocket( addr.sin_addr.s_addr, addr.sin_port, 0 );
        assert( fd >= 0 );
        #else
        fd = socket( PF_INET, SOCK_DGRAM, 0 );
        assert( fd != INVALID_SOCKET );
        // set blocking/nonblocking
        this->set_socket_blocking( fd, wait );
        retval = connect( fd, (sockaddr*)&addr, sizeof( addr ) );
        assert( retval != SOCKET_ERROR );
        // set blocking/nonblocking
        this->set_socket_blocking( fd, waitForData );
        #endif // ifdef NBMINGW
        this->lastReadTime = TIME2DOUBLE( this->get_time( ) );
        fprintf( stderr, "...connected\n" );
    }
    
    // repair options
    options[ i ] = ',';
    
    return fd;
}

// read from device (buf = buffer into which to read, count = max number of bytes to read,
//   uses maxInterval and wait, returns number of bytes read or -1 on error)
//NOTE: SOCK_DGRAM does not return everything in buffer, only last message; this function compensates for that by looping the recv (but does not read part of a message, which would cause the rest of the message to be discarded)
ssize_t UdpCharDevice::read( void *buf, size_t count )
{
    ssize_t retval = 0;
    ssize_t sumretval = 0;
    bool peek = false;
    bool cont;
    while( (size_t)sumretval < count )
    {
        if( !this->connected( ) )
        {
            //this->close_client( );
            if( this->open( ) == -1 && !wait )
            {
                retval = -1;
                break;
            }
            if( !this->connected( ) )
                continue;
        }
        #ifdef WIN32
        // recv() returns an error if the message is too large to fit in the buffer
        retval = ::recv( fd, (read_write_type)&(((char*)buf)[sumretval]), count - sumretval, peek ? MSG_PEEK : 0 ); //NOTE: was ::read()
        #elif defined(NBMINGW)
        if( peek )
        {
            retval = peek_socket_udp( fd, 0 );
        }
        else
        {
            if( !dataavail( fd ) && !waitForData )
            {
                retval = 0;
                break;
            }
            retval = ::read( fd, (read_write_type)&(((char*)buf)[sumretval]), count - sumretval );
        }
        #else // ifdef WIN32
        // MSG_TRUNC causes recv() to return the actual length of the message if it is too large to fit in the buffer
        retval = ::recv( fd, (read_write_type)&(((char*)buf)[sumretval]), count - sumretval, ( peek ? MSG_PEEK : 0 ) | MSG_TRUNC ); //NOTE: was ::read()
        #endif // ifdef WIN32
        cont = test_recv_retval( retval, count - (size_t)sumretval, true, ( sumretval == 0 ) );
        if( !peek && retval > 0 )
        { // data is OK
            sumretval += retval;
        }
        if( !cont )
            break;
        peek = !peek;
    }
    if( sumretval > 0 )
        retval = sumretval;
    else if( retval == SOCKET_ERROR )
        retval = -1;
    return retval;
}

// write to device (buf = buffer from which to write, count = number number of bytes to write,
//   uses wait, returns number of bytes written or -1 on error)
ssize_t UdpCharDevice::write( const void *buf, size_t count )
{
    ssize_t retval;
    while( 1 )
    {
        if( !this->connected( ) )
        {
            //this->close_client( );
            if( this->open( ) == -1 && !wait )
            {
                retval = -1;
                break;
            }
            if( !this->connected( ) )
                continue;
        }
        #ifdef WIN32
        retval = ::send( fd, (const read_write_type)buf, count, 0 );
        #elif defined(NBMINGW)
        retval = ::write( fd, (const read_write_type)buf, count );
        #else // ifdef WIN32
        retval = ::send( fd, (const read_write_type)buf, count, MSG_NOSIGNAL );
        #endif // ifdef WIN32
        if( !test_send_retval( retval ) )
            break;
    }
    if( retval == SOCKET_ERROR )
        retval = -1;
    return retval;
}

// close device, called last time by destructor (returns 0 on success or -1 on error)
int UdpCharDevice::close( void )
{
    int retval = 0;
    retval = this->close_socket( fd );
    return retval;
}

// close connection (returns 0 on success or -1 on error)
int UdpCharDevice::close_connection( void )
{
    int retval = 0;
    retval = this->close_socket( fd );
    return retval;
}

// whether currently connected
bool UdpCharDevice::connected( void )
{
    return ( fd != INVALID_SOCKET );
}
