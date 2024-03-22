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
// chardevicebase.cpp
//
//    Base functionality that is used by many chardevices.
//
// authors
//
//    Todd R. Templeton <ttemplet@eecs.berkeley.edu>
//
//----------------------------------------------------------------------------------


#include "chardevicebase.h"
#include "chardevicecommon.h"


namespace {
    
    // default function call for getting the current time (in microseconds)
    uint64_t (*default_get_time)( void ) = get_elapsed_time;
    
} // namespace


#ifdef WIN32

void usleep( unsigned long usec )
{
  unsigned long msec = usec / 1000;
  if( msec == 0 && usec > 0 )
    msec = 1;
  Sleep( msec );
}

#elif defined(NBMINGW)

void usleep( unsigned long usec )
{
  unsigned long ticks = ( usec * TICKS_PER_SECOND ) / 1000000;
  if( ticks == 0 && usec > 0 )
    ticks = 1;
  OSTimeDly( ticks );
}

#endif


#ifdef NBMINGW

uint16_t htons( uint16_t a )
{
    return HTONS( a );
}


uint16_t ntohs( uint16_t a )
{
    return NTOHS( a );
}


uint32_t htonl( uint32_t a )
{
    return HTONL( a );
}


uint32_t ntohl( uint32_t a )
{
    return NTOHL( a );
}

#endif


#if __BYTE_ORDER == __BIG_ENDIAN

uint64_t htonll( uint64_t a )
{
    return a;
}


uint64_t ntohll( uint64_t a )
{
    return a;
}

#else

//FIXME: check these...
uint64_t htonll( uint64_t a )
{
    return ( ( (uint64_t)htonl( a ) ) << 32 ) + htonl( a >> 32 );
}


uint64_t ntohll( uint64_t a )
{
    return ( ( (uint64_t)ntohl( a ) ) << 32 ) + ntohl( a >> 32 );
}

#endif


#ifdef WIN32

// get elapsed time, first call returns 0
uint64_t get_elapsed_time( void )
{
    static bool getTimeInit = false;
    static uint64_t start;
    struct _timeb tv;

    if( !getTimeInit )
    {
        _ftime( &tv );
        start = 1000000 * (uint64_t)tv.time + 1000 * (uint64_t)tv.millitm;
        getTimeInit = true;
        return 0;
    }
        
    _ftime( &tv );
    return ( 1000000 * (uint64_t)tv.time + 1000 * (uint64_t)tv.millitm - start );
}

#elif defined(NBMINGW)

// get elapsed time, first call returns 0
uint64_t get_elapsed_time( void )
{
    static bool getTimeInit = false;
    static uint64_t start;

    if( !getTimeInit )
    {
        start = 1000000 * (uint64_t)Secs + (uint64_t)(TimeTick % TICKS_PER_SECOND) * 1000000 / TICKS_PER_SECOND;
        getTimeInit = true;
        return 0;
    }
    
    return ( 1000000 * (uint64_t)Secs + (uint64_t)(TimeTick % TICKS_PER_SECOND) * 1000000 / TICKS_PER_SECOND - start );
}

#else

// get elapsed time, first call returns 0
uint64_t get_elapsed_time( void )
{
    static bool getTimeInit = false;
    static uint64_t start;
    struct timeval tv;
    struct timezone tz;

    if( !getTimeInit )
    {
        gettimeofday( &tv, &tz );
        start = 1000000 * (uint64_t)tv.tv_sec + (uint64_t)tv.tv_usec;
        getTimeInit = true;
        return 0;
    }
        
    gettimeofday( &tv, &tz );
    return ( 1000000 * (uint64_t)tv.tv_sec + (uint64_t)tv.tv_usec - start );
}

#endif


// set default function call for getting the current time (in microseconds)
/*static*/ void CharDevice::set_default_time_function( uint64_t (*_get_time)( void ) )
{
    default_get_time = _get_time;
}


// constructor
CharDevice::CharDevice( ) : wait( true ), maxInterval( -1.0 ), waitForData( false ), get_time( default_get_time )
{
}
