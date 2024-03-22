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
// socketchardevicebase.cpp
//
//    Base class for cross-platform interfaces to sockets (TCP and UDP).
//
// authors
//
//    Todd R. Templeton <ttemplet@eecs.berkeley.edu>
//
//----------------------------------------------------------------------------------


#include "socketchardevicebase.h"
#include "chardevicecommon.h"


#ifdef NBMINGW

#include <iointernal.h>

struct UdpSocketDataSet
{
    OS_FIFO the_fifo;
    int fd;
    IPADDR address;
    WORD rxport;
    WORD lport;
    WORD rport;
};

class UDPPacketNoFree : public UDPPacket
{
public:

    UDPPacketNoFree( PoolPtr p ) : UDPPacket( p )
    {
    }
    
    ~UDPPacketNoFree( )
    {
        // make sure that UDPPacket::~UDPPacket() does not free m_p
        m_p = 0;
    }
};

// determine how many bytes are in next message, and from where it was sent
/*static*/ ssize_t SocketCharDevice::peek_socket_udp( SOCKET s, sockaddr_in *addr )
{
    ssize_t retval = 0;
    pool_buffer *p;
    UdpSocketDataSet *d = (UdpSocketDataSet*)GetExtraData( s ); 
    if( d == 0 )
        return UDP_ERR_NOSUCH_SOCKET; 
    if( d->rxport == 0 )
        return UDP_ERR_NOTOPEN_TO_READ;
    USER_ENTER_CRITICAL()
    if( ( p = (pool_buffer*)(d->the_fifo.pHead) ) != 0 )
    {
        UDPPacketNoFree up( p );
        if( up.Validate( ) )
        {
            retval = up.GetDataSize( );
            if( addr )
            {
                addr->sin_addr.s_addr = up.GetSourceAddress( );
                addr->sin_port = up.GetSourcePort( );
            }
        }
    }
    USER_EXIT_CRITICAL()
    return retval;
}

#endif


// initialize socket library, e.g. Windows WGA
/*static*/ void SocketCharDevice::init_socket_library( void )
{
    #ifdef WIN32
    WORD versionRequested = MAKEWORD( 1, 1 );
    WSADATA wd;
    WSAStartup( versionRequested, &wd );
    assert( wd.wVersion == versionRequested );
    #endif
}


// clean up socket library, e.g. Windows WGA
/*static*/ void SocketCharDevice::cleanup_socket_library( void )
{
    #ifdef WIN32
    WSACleanup( );
    #endif
}


// set socket as blocking or nonblocking
/*static*/ void SocketCharDevice::set_socket_blocking( SOCKET s, bool blocking )
{
    int __attribute__((unused)) retval = 0;
    #ifdef WIN32
    unsigned long nonblocking = blocking ? 0 : 1;
    retval = ioctlsocket( s, FIONBIO, &nonblocking );
    #elif defined(NBMINGW)
    //NOTE: cannot set socket as nonblocking on NetBurner, but can check if data is available to read
    assert( 0 );
    #else
    if( blocking )
        retval = fcntl( s, F_SETFL, fcntl( s, F_GETFL ) & ~O_NONBLOCK ); //FIXME: check this
    else
        retval = fcntl( s, F_SETFL, fcntl( s, F_GETFL ) | O_NONBLOCK );
    #endif
    assert( retval != SOCKET_ERROR );
}


// set socket to be able to reuse a local address
/*static*/ void SocketCharDevice::set_socket_server_reuse_address( SOCKET s )
{
    int __attribute__((unused)) retval = 0;
    #ifndef NBMINGW
    int reuseaddr = 1;
    #endif
    #ifdef WIN32
    retval = setsockopt( s, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuseaddr, sizeof( reuseaddr ) );
    #elif defined(NBMINGW)
    //FIXME: this is probably the default behavior
    #else // ifdef WIN32
    retval = setsockopt( s, SOL_SOCKET, SO_REUSEADDR, (const void*)&reuseaddr, sizeof( reuseaddr ) );
    #endif // ifdef WIN32
    assert( retval != SOCKET_ERROR );
}


// set socket to not use the Nagle algorithm
/*static*/ void SocketCharDevice::set_socket_tcp_no_delay( SOCKET s )
{
    int __attribute__((unused)) retval = 0;
    #ifndef NBMINGW
    int nodelay = 1;
    #endif
    #ifdef WIN32
    retval = setsockopt( s, IPPROTO_TCP, TCP_NODELAY, (const char*)&nodelay, sizeof( nodelay ) );
    #elif defined(NBMINGW)
    setsockoption( s, SO_NONAGLE );
    #else // ifdef WIN32
    retval = setsockopt( s, SOL_TCP, TCP_NODELAY, (const void*)&nodelay, sizeof( nodelay ) );
    #endif // ifdef WIN32
    assert( retval != SOCKET_ERROR );
}


// close a socket (returns 0 on success or -1 on error)
/*static*/ int SocketCharDevice::close_socket( SOCKET &s )
{
    int retval = 0;
    if( s != INVALID_SOCKET )
    {
        #ifdef WIN32
        retval = closesocket( s );
        if( retval == SOCKET_ERROR )
            retval = -1;
        #else
        retval = ::close( s );
        #endif
        s = INVALID_SOCKET;
    }
    return retval;
}


// fill port and host in sockaddr_in structure
/*static*/ void SocketCharDevice::fill_sockaddr_in( sockaddr_in &addr, const char *host, int port )
{
    #ifndef NBMINGW
    addr.sin_family = AF_INET;
    #endif
    if( port >= 0 )
    {
        #ifdef NBMINGW
        addr.sin_port = port;
        #else
        addr.sin_port = htons( port );
        #endif
    }
    if( host )
    {
        #ifdef WIN32
        addr.sin_addr.s_addr = inet_addr( host );
        #elif defined(NBMINGW)
        addr.sin_addr.s_addr = AsciiToIp( host );
        #else
        int __attribute__((unused)) retval;
        retval = inet_pton( AF_INET, host, &addr.sin_addr );
        assert( retval > 0 );
        #endif
    }
    else
        addr.sin_addr.s_addr = INADDR_ANY;
}


// set whether to wait for at least one byte before returning from a read
void SocketCharDevice::set_wait_for_data( bool _waitForData )
{
    waitForData = _waitForData;
    
    #ifndef NBMINGW
    if( fd != INVALID_SOCKET )
        this->set_socket_blocking( fd, _waitForData );
    #endif
}


// test return value from read/recv; returns whether to continue reading
bool SocketCharDevice::test_recv_retval( ssize_t& retval, ssize_t spaceRemainingInBuffer, bool messageProtocol /*= false*/, bool allowPartialMessageRead /*= false*/ )
{
    bool cont = true;
    bool readError = false, maxIntervalExpired = false; //, messageTooLarge = false;
    double t;
    if( retval == 0 && waitForData ) //FIXME: get rid of this
        fprintf( stderr, "Waiting for data, but received none.\n" );
    #ifdef NBMINGW
    if( retval < 0 )
    {
        readError = true;
    }
    #else // ifdef NBMINGW
    if( retval == SOCKET_ERROR )
    {
        #ifdef WIN32
        int lerror = WSAGetLastError();
        switch( lerror )
        {
        case WSAEWOULDBLOCK:
            assert( !waitForData );
            retval = 0;
            break;
        case WSAEMSGSIZE: // UDP, when buffer is not large enough for latest message (but, like in Linux, as much of the message as possible is read and the rest is discarded)
            assert( messageProtocol );
            retval = allowPartialMessageRead ? spaceRemainingInBuffer : 0;
            //messageTooLarge = true;
            break;
        case WSAENETRESET:
        case WSAENOTCONN:
        case WSAENOTSOCK:
        case WSAEHOSTUNREACH:
        case WSAECONNABORTED:
        case WSAECONNRESET:
        case WSAETIMEDOUT:
            readError = true;
            break;
        default:
            fprintf( stderr, "Unexpected read error: %d\n", lerror );
            assert( 0 );
            break;
        }
        #else
        switch( errno )
        {
        case EAGAIN:
        #if EAGAIN != EWOULDBLOCK
        case EWOULDBLOCK:
        #endif
            assert( !waitForData );
            retval = 0;
            break;
        case EPIPE:
        case ECONNRESET:
        case EBADF:
        case ENOTCONN:
        case ENOTSOCK:
            readError = true;
            break;
        default:
            fprintf( stderr, "Unexpected read error: %d\n", errno );
            assert( 0 );
            break;
        }
        #endif
    }
    #endif // ifdef NBMINGW
    else if( retval > spaceRemainingInBuffer )
    {
        #ifdef WIN32
        assert( 0 );
        #else // ifdef WIN32
        assert( messageProtocol );
        retval = allowPartialMessageRead ? spaceRemainingInBuffer : 0;
        //messageTooLarge = true;
        #endif // ifdef WIN32
    }
    if( !readError )
    {
        t = TIME2DOUBLE( this->get_time( ) );
        if( retval == 0 && maxInterval > 0.0 && ( t - this->lastReadTime ) > maxInterval )
            maxIntervalExpired = true;
        else if( retval > 0 )
            this->lastReadTime = t;
    }
    if( readError || maxIntervalExpired )
    {
        this->close_connection( );
        if( this->open( ) == -1 && !wait )
            cont = false;
    }
    if( retval == 0 && !waitForData )
        cont = false; // received no data, but are not supposed to wait
    else if( retval > 0 && !( messageProtocol && retval < spaceRemainingInBuffer ) )
        cont = false; // received data, should not wait for more
    return cont;
}


// test return value from write/send; returns whether to continue writing
bool SocketCharDevice::test_send_retval( ssize_t& retval )
{
    bool cont = true;
    bool writeError = false;
    #ifdef NBMINGW
    if( retval < 0 )
    {
        writeError = true;
    }
    #else // ifdef NBMINGW
    if( retval == SOCKET_ERROR )
    {
        #ifdef WIN32
        int lerror = WSAGetLastError();
        switch( lerror )
        {
        case WSAEWOULDBLOCK:
            retval = 0;
            break;
        case WSAENETRESET:
        case WSAENOTCONN:
        case WSAENOTSOCK:
        case WSAEHOSTUNREACH:
        case WSAECONNABORTED:
        case WSAECONNRESET:
        case WSAETIMEDOUT:
            writeError = true;
            break;
        default:
            fprintf( stderr, "Unexpected write error: %d\n", lerror );
            assert( 0 );
            break;
        }
        #else
        switch( errno )
        {
        case EAGAIN:
        #if EAGAIN != EWOULDBLOCK
        case EWOULDBLOCK:
        #endif
            retval = 0;
            break;
        case EPIPE:
        case ECONNRESET:
        case EBADF:
        case ENOTCONN:
        case ENOTSOCK:
            writeError = true;
            break;
        default:
            fprintf( stderr, "Unexpected write error: %d\n", errno );
            assert( 0 );
            break;
        }
        #endif
    }
    #endif // ifdef NBMINGW
    if( writeError )
    {
        this->close_connection( );
        if( this->open( ) == -1 && !wait )
            cont = false;
    }
    else
        cont = false; // if we successfully wrote something (or nothing), the OS' write buffer is full--the application needs to deal with this
    return cont;
}
