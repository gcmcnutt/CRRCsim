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
// tcpchardevice.cpp
//
//    Cross-platform interface to a TCP client.
//
// authors
//
//    Todd R. Templeton <ttemplet@eecs.berkeley.edu>
//
//----------------------------------------------------------------------------------


#include "tcpchardevice.h"
#include "chardevicecommon.h"


// open device, called first time by constructor (uses wait, returns -1 on error)
int TcpCharDevice::open( void )
{
    int i;
    int len = strlen( options );
    char *host;
    int port;
    sockaddr_in addr;
    #ifdef WIN32
    int lerror;
    #endif
    bool success = false;
    #ifndef NBMINGW
    int retval;
    #endif
    
    // find host
    for( i = 0; i < len && options[ i ] != ','; i++ );
    assert( ( i + 1 ) < len );
    options[ i ] = '\0';
    host = options;
    // find port
    port = atoi( &options[ i + 1 ] );
    
    while( 1 )
    {
        if( !connecting )
        {
            fprintf( stderr, "connecting to %s port %d\n", host, port );
    
            this->fill_sockaddr_in( addr, host, port );
            #ifdef NBMINGW
            connectsock = connect( addr.sin_addr.s_addr, 0, addr.sin_port, wait ? 0 : ( TICKS_PER_SECOND / 2 ) ); //FIXME: this always waits for a while
            assert( connectsock >= -1 );
            if( connectsock == -1 ) // -1 means timeout
                connectsock = INVALID_SOCKET;
            else
              success = true;
            #else // ifdef NBMINGW
            connectsock = socket( PF_INET, SOCK_STREAM, 0 );
            assert( connectsock != INVALID_SOCKET );
            // set blocking/nonblocking
            this->set_socket_blocking( connectsock, wait );
            retval = connect( connectsock, (sockaddr*)&addr, sizeof( addr ) );
            if( retval != SOCKET_ERROR )
                connecting = true;
            #ifdef WIN32
            else if( ( lerror = WSAGetLastError() ) == WSAEWOULDBLOCK )
            #else
            else if( errno == EINPROGRESS /*|| errno == 0*/ ) //FIXME: on stargate, errno is 0
            #endif
            {
                assert( !wait );
                connecting = true;
            }
            if( !connecting )
                this->close_connection( );
            #endif // ifdef NBMINGW
        }
        if( connecting )
        {
            #ifdef NBMINGW
            assert( 0 );
            #else // ifdef NBMINGW
            while( 1 )
            {
                struct timeval tv;
                fd_set wset, eset;
                #ifndef WIN32
                int sockoptVal;
                socklen_t sockoptValSize = sizeof( int );
                #endif
                FD_ZERO( &wset );
                FD_SET( connectsock, &wset );
                FD_ZERO( &eset );
                #ifdef WIN32
                FD_SET( connectsock, &eset );
                #endif
                tv.tv_sec = 0;
                tv.tv_usec = wait ? 100000 : 0;
                retval = select( connectsock + 1, 0, &wset, &eset, &tv );
                if( retval == 0 )
                {
                    if( wait )
                        continue;
                    else
                        break;
                }
                #ifdef WIN32
                else if( retval < 0 || FD_ISSET( connectsock, &eset ) )
                #else
                else if( retval < 0 || getsockopt( connectsock, SOL_SOCKET, SO_ERROR, &sockoptVal, &sockoptValSize ) < 0 || sockoptVal != 0 )
                #endif
                {
                    this->close( );
                    connecting = false;
                    break;
                }
                success = true;
                break;
            }
            #endif // ifdef NBMINGW
        }
        if( success )
        {
            fd = connectsock;
            connectsock = INVALID_SOCKET;
            connecting = false;
            #ifndef NBMINGW
            // set blocking/nonblocking
            this->set_socket_blocking( fd, waitForData );
            #endif
            this->set_socket_tcp_no_delay( fd );
            this->lastReadTime = TIME2DOUBLE( this->get_time( ) );
            fprintf( stderr, "...connected\n" );
            break;
        }
        else if( wait )
        { // something bad happened; wait and then try again
            usleep( 100000 );
            continue;
        }
        else
            break;
    }
    
    // repair options
    options[ i ] = ',';
    
    return ( fd != INVALID_SOCKET ? (int)fd : -1 );
}

// read from device (buf = buffer into which to read, count = max number of bytes to read,
//   uses maxInterval and wait, returns number of bytes read or -1 on error)
ssize_t TcpCharDevice::read( void *buf, size_t count )
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
ssize_t TcpCharDevice::write( const void *buf, size_t count )
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
int TcpCharDevice::close( void )
{
    int retval = 0;
    retval = this->close_socket( fd );
    this->close_socket( connectsock );
    connecting = false;
    return retval;
}

// close connection (returns 0 on success or -1 on error)
int TcpCharDevice::close_connection( void )
{
    int retval = 0;
    retval = this->close_socket( fd );
    if( !connecting )
        this->close_socket( connectsock );
    return retval;
}

// whether currently connected
bool TcpCharDevice::connected( void )
{
    return ( fd != INVALID_SOCKET );
}
