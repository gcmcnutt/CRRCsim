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
// tcpserverchardevice.cpp
//
//    Cross-platform interface to a single-connection TCP server.
//
// authors
//
//    Todd R. Templeton <ttemplet@eecs.berkeley.edu>
//
//----------------------------------------------------------------------------------


#include "tcpserverchardevice.h"
#include "chardevicecommon.h"


// open device, called first time by constructor (uses wait, returns -1 on error)
int TcpServerCharDevice::open( void )
{
    int i;
    int len = strlen( options );
    char *client;
    char defaultmask[16] = "255.255.255.255";
    char *mask = defaultmask;
    int maskpos = -1;
    int port;
    sockaddr_in addr, addr2, addrmask;
    #ifndef NBMINGW
    socklen_t addrlen = sizeof( addr );
    #endif
    int __attribute__((unused)) retval;
    
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
    
    if( !listening )
    {
        fprintf( stderr, "opening port %d to listen for connections, waiting for client %s/%s\n", port, client, mask );
        
        this->fill_sockaddr_in( addr, 0, port );
        #ifdef NBMINGW
        listensock = listen( INADDR_ANY, addr.sin_port, 1 );
        assert( listensock >= 0 );
        #else // ifdef NBMINGW
        listensock = socket( PF_INET, SOCK_STREAM, 0 );
        assert( listensock != INVALID_SOCKET );
        // set blocking/nonblocking
        this->set_socket_blocking( listensock, wait );
        this->set_socket_server_reuse_address( listensock );
        retval = bind( listensock, (sockaddr*)&addr, addrlen );
        assert( retval != SOCKET_ERROR );
        retval = listen( listensock, 1 );
        assert( retval != SOCKET_ERROR );
        #endif // ifdef NBMINGW
        listening = true;
    }
    this->fill_sockaddr_in( addr2, client, -1 );
    this->fill_sockaddr_in( addrmask, mask, -1 );
    while( 1 )
    {
        #ifdef NBMINGW
        if( !wait )
        {
            fd_set rset, eset;
            FD_ZERO( &rset );
            FD_SET( listensock, &rset );
            FD_ZERO( &eset );
            FD_SET( listensock, &eset );
            retval = ZeroWaitSelect( FD_SETSIZE, &rset, 0, &eset );
            if( retval < 0 || FD_ISSET( listensock, &eset ) )
            {
                this->close( );
                listening = false;
            }
            else if( retval == 0 || !FD_ISSET( listensock, &rset ) )
                break;
        }
        fd = accept( listensock, &addr.sin_addr.s_addr, 0, 0 );
        if( fd < 0 )
            fd = INVALID_SOCKET;
        #else // ifdef NBMINGW
        fd = accept( listensock, (sockaddr*)&addr, &addrlen );
        #endif // ifdef NBMINGW
        if( fd == INVALID_SOCKET )
        {
            #ifdef WIN32
            int lerror = WSAGetLastError();
            assert( lerror == WSAEWOULDBLOCK || lerror == WSAECONNRESET || lerror == WSAENETDOWN );
            #elif defined(NBMINGW)
            #else // ifdef WIN32
            assert( errno == EAGAIN || errno == EWOULDBLOCK || errno == ENETDOWN || errno == EPROTO || errno == ENOPROTOOPT || errno == EHOSTDOWN || errno == ENONET || errno == EHOSTUNREACH || errno == EOPNOTSUPP || errno == ENETUNREACH );
            #endif // ifdef WIN32
            if( wait )
            {
                #ifdef WIN32
                assert( lerror != WSAEWOULDBLOCK );
                #elif defined(NBMINGW)
                #else // ifdef WIN32
                assert( errno != EAGAIN && errno != EWOULDBLOCK );
                #endif // ifdef WIN32
                usleep( 100000 );
                continue;
            }
            else
                break;
        }
        if( !( ( addr.sin_addr.s_addr ^ addr2.sin_addr.s_addr ) & addrmask.sin_addr.s_addr ) ) // check address of client
        {
            // set blocking/nonblocking
            #ifndef NBMINGW
            this->set_socket_blocking( fd, waitForData );
            #endif
            this->set_socket_tcp_no_delay( fd );
            //::close( listensock );
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
ssize_t TcpServerCharDevice::read( void *buf, size_t count )
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
        retval = ::recv( fd, (read_write_type)buf, count, 0 ); //NOTE: was ::read()
        #elif defined(NBMINGW)
        if( !dataavail( fd ) && !waitForData )
        {
            retval = 0;
            break;
        }
        retval = ::read( fd, (read_write_type)buf, count );
        #else // ifdef WIN32
        retval = ::recv( fd, (read_write_type)buf, count, 0 ); //NOTE: was ::read()
        #endif // ifdef WIN32
        if( !test_recv_retval( retval, count, false ) )
            break;
    }
    if( retval == SOCKET_ERROR )
        retval = -1;
    return retval;
}

// write to device (buf = buffer from which to write, count = number number of bytes to write,
//   uses wait, returns number of bytes written or -1 on error)
ssize_t TcpServerCharDevice::write( const void *buf, size_t count )
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
int TcpServerCharDevice::close( void )
{
    int retval = 0;
    retval = this->close_socket( fd );
    this->close_socket( listensock );
    listening = false;
    return retval;
}

// close current client (returns 0 on success or -1 on error)
int TcpServerCharDevice::close_client( void )
{
    int retval = 0;
    retval = this->close_socket( fd );
    return retval;
}

// close connection (returns 0 on success or -1 on error)
int TcpServerCharDevice::close_connection( void )
{
    return this->close_client( );
}

// whether currently connected
bool TcpServerCharDevice::connected( void )
{
    return ( fd != INVALID_SOCKET );
}
