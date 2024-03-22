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
// udpserverchardevice.cpp
//
//    Cross-platform interface to a single-connection UDP server.
//
// authors
//
//    Todd R. Templeton <ttemplet@eecs.berkeley.edu>
//
//----------------------------------------------------------------------------------


#include "udpserverchardevice.h"
#include "chardevicecommon.h"


// open device, called first time by constructor (uses wait, returns -1 on error)
int UdpServerCharDevice::open( void )
{
    int i;
    int len = strlen( options );
    char *client;
    char defaultmask[16] = "255.255.255.255";
    char *mask = defaultmask;
    int maskpos = -1;
    int port;
    //int listensock;
    sockaddr_in addr, addr2, addrmask;
    #ifndef NBMINGW
    socklen_t addrlen = sizeof( addr );
    #endif
    int retval;
    
    // find client
    for( i = 0; i < len && options[ i ] != ','; i++ )
    {
        if( options[ i ] == '/' && ( i + 1 ) < len && maskpos < 0 )
        {
            options[ i ] = '\0';
            maskpos = i + 1;
            mask = &options[ maskpos ];
        }
    }
    assert( ( i + 1 ) < len );
    options[ i ] = '\0';
    client = options;
    // find port
    port = atoi( &options[ i + 1 ] );

    if( fd == INVALID_SOCKET )
    {
        fprintf( stderr, "opening port %d to listen for connections, waiting for client %s/%s\n", port, client, mask );
        
        this->fill_sockaddr_in( addr, 0, port );
        #ifdef NBMINGW
        fd = CreateRxUdpSocket( addr.sin_port );
        assert( fd >= 0 );
        #else
        fd = socket( PF_INET, SOCK_DGRAM, 0 );
        assert( fd != INVALID_SOCKET );
        // set nonblocking
        this->set_socket_blocking( fd, wait );
        this->set_socket_server_reuse_address( fd );
        retval = bind( fd, (sockaddr*)&addr, addrlen );
        assert( retval != SOCKET_ERROR );
        #endif
        
        connectedClientAddrValid = false;
    }
    this->fill_sockaddr_in( addr2, client, -1 );
    this->fill_sockaddr_in( addrmask, mask, -1 );
    while( 1 )
    {
        #ifdef NBMINGW
        retval = peek_socket_udp( fd, &addr );
        if( retval <= 0 )
        #else
        char buf[10];
        retval = ::recvfrom( fd, buf, 10, MSG_PEEK, (sockaddr*)&addr, &addrlen );
        if( retval == SOCKET_ERROR )
        #endif
        {
            #ifdef WIN32
            int lerror = WSAGetLastError();
            assert( lerror == WSAEWOULDBLOCK || lerror == WSAEMSGSIZE || lerror == WSAENETDOWN );
            if( lerror != WSAEMSGSIZE )
            #elif defined(NBMINGW)
            #else // ifdef WIN32
            assert( errno == EAGAIN || errno == EWOULDBLOCK || errno == ENETDOWN || errno == EPROTO || errno == ENOPROTOOPT || errno == EHOSTDOWN || errno == ENONET || errno == EHOSTUNREACH || errno == EOPNOTSUPP || errno == ENETUNREACH );
            #endif // ifdef WIN32
            {
                if( wait )
                {
                    #ifdef WIN32
                    assert( lerror != WSAEWOULDBLOCK );
                    #elif defined(NBMINGW)
                    assert( retval == 0 );
                    #else // ifdef WIN32
                    assert( errno != EAGAIN && errno != EWOULDBLOCK );
                    #endif // ifdef WIN32
                    usleep( 100000 );
                    continue;
                }
                else
                    break;
            }
        }
        connectedClientAddr = addr;
        connectedClientAddrValid = true;
        if( !( ( addr.sin_addr.s_addr ^ addr2.sin_addr.s_addr ) & addrmask.sin_addr.s_addr ) ) // check address of client
        {
            #ifdef NBMINGW
            this->close_socket( fd );
            fd = CreateRxTxUdpSocket( addr2.sin_addr.s_addr, addr2.sin_port, addr.sin_port );
            assert( fd >= 0 );
            #endif
            this->lastReadTime = TIME2DOUBLE( this->get_time( ) );
            fprintf( stderr, "...connected\n" );
            break;
        }
        else {
            fprintf( stderr, "rejecting connection from incorrect client\n" );
            this->close_client( );
        }
    }
    
    // repair options
    if( maskpos > 0 )
        options[ maskpos - 1 ] = '/';
    options[ i ] = ',';
    
    return ( this->connected( ) ? (int)fd : -1 );
}

// read from device (buf = buffer into which to read, count = max number of bytes to read,
//   uses maxInterval and wait, returns number of bytes read or -1 on error)
//NOTE: SOCK_DGRAM does not return everything in buffer, only last message; this function compensates for that by looping the recv (but does not read part of a message from an allowed client, which would cause the rest of the message to be discarded)
ssize_t UdpServerCharDevice::read( void *buf, size_t count )
{
    sockaddr_in recvAddr;
    #ifndef NBMINGW
    socklen_t addrlen;
    #endif
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
        #ifndef NBMINGW
        addrlen = sizeof( recvAddr );
        #endif
        #ifdef WIN32
        // recvfrom() returns an error if the message is too large to fit in the buffer
        retval = ::recvfrom( fd, (read_write_type)&(((char*)buf)[sumretval]), count - sumretval, peek ? MSG_PEEK : 0, (sockaddr*)&recvAddr, &addrlen ); //NOTE: was ::read()
        #elif defined(NBMINGW)
        if( peek )
        {
            retval = peek_socket_udp( fd, &recvAddr );
        }
        else
        {
            if( !dataavail( fd ) && !waitForData )
            {
                retval = 0;
                break;
            }
            uint16_t localPort;
            retval = ::recvfrom( fd, (BYTE*)&(((char*)buf)[sumretval]), count - sumretval, &recvAddr.sin_addr.s_addr, &localPort, &recvAddr.sin_port );
        }
        #else // ifdef WIN32
        // MSG_TRUNC causes recv() to return the actual length of the message if it is too large to fit in the buffer
        retval = ::recvfrom( fd, (read_write_type)&(((char*)buf)[sumretval]), count - sumretval, ( peek ? MSG_PEEK : 0 ) | MSG_TRUNC, (sockaddr*)&recvAddr, &addrlen ); //NOTE: was ::read()
        #endif // ifdef WIN32
        cont = test_recv_retval( retval, count - (size_t)sumretval, true, ( sumretval == 0 ) );
        if( retval > 0 )
        {
            assert( connectedClientAddrValid );
            if( recvAddr.sin_addr.s_addr ^ connectedClientAddr.sin_addr.s_addr )
            { // received data from different address than the one that first connected
                if( !peek )
                    fprintf( stderr, "rejecting connection from additional client\n" );
                retval = 0;
            }
        }
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
ssize_t UdpServerCharDevice::write( const void *buf, size_t count )
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
        retval = ::sendto( fd, (const read_write_type)buf, count, 0, (sockaddr*)&connectedClientAddr, sizeof( connectedClientAddr ) );
        #elif defined(NBMINGW)
        retval = ::sendto( fd, (BYTE*)buf, count, connectedClientAddr.sin_addr.s_addr, connectedClientAddr.sin_port );
        #else
        retval = ::sendto( fd, (const read_write_type)buf, count, MSG_NOSIGNAL, (sockaddr*)&connectedClientAddr, sizeof( connectedClientAddr ) );
        #endif
        if( !test_send_retval( retval ) )
            break;
    }
    if( retval == SOCKET_ERROR )
        retval = -1;
    return retval;
}

// close device, called last time by destructor (returns 0 on success or -1 on error)
int UdpServerCharDevice::close( void )
{
    int retval = 0;
    retval = this->close_socket( fd );
    connectedClientAddrValid = false;
    return retval;
}

// close current client (returns 0 on success or -1 on error)
int UdpServerCharDevice::close_client( void )
{
    int retval = 0;
    retval = this->close_socket( fd );
    connectedClientAddrValid = false;
    return retval;
}

// close connection (returns 0 on success or -1 on error)
int UdpServerCharDevice::close_connection( void )
{
    return this->close_client( );
}

// whether currently connected
bool UdpServerCharDevice::connected( void )
{
    return ( fd != INVALID_SOCKET && connectedClientAddrValid );
}
