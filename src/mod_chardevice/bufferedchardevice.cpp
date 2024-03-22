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
// bufferedchardevice.cpp
//
//    A "buffered character device" that serves as a base class for message-passing
//    architectures that receive a single type of message that is no more than 256
//    characters in length (transparently over any one of chardevice's supported
//    interfaces).
//
// authors
//
//    Todd R. Templeton <ttemplet@eecs.berkeley.edu>
//
//----------------------------------------------------------------------------------


#include "bufferedchardevice.h"
#include "chardevicewrapper.h"
#include "chardevicecommon.h"


// initialization (device = device options string, wait = whether to wait for connection/reconnection)
void BufferedCharDevice::init( const char *device, bool wait /* = true */ )
{
    charDevice = new CharDeviceWrapper( device, wait );
    deleteCharDevice = true;
    
    circbufSize = 256;
    circbuf = new char[ circbufSize ];
    circbufStart = 0;
    circbufNext = 0;
    circbufLength = 0;
    
    //numUpdates = 0;
}

// initialization (_charDevice = CharDevice to use)
void BufferedCharDevice::init( CharDevice *_charDevice )
{
    charDevice = _charDevice;
    deleteCharDevice = false;
    
    circbufSize = 256;
    circbuf = new char[ circbufSize ];
    circbufStart = 0;
    circbufNext = 0;
    circbufLength = 0;
    
    //numUpdates = 0;
}

// cleanup
void BufferedCharDevice::cleanup( void )
{
    if( charDevice && deleteCharDevice )
        delete charDevice;
            
    if( circbuf )
        delete[] circbuf;
}

// read bytes into circular buffer (uses charDevice's maxInterval and wait)
void BufferedCharDevice::read_into_buffer( void )
{
    int res, endspace;
    
    while( ( res = charDevice->read( /*ser_fd,*/ &circbuf[ circbufNext ], ( endspace = circbufSize - (int)circbufNext ) ) ) > 0 )
    {
        //fprintf( stderr, "read %d bytes\n", res );
        assert( res >= 0 && res <= circbufSize );
        assert( res <= endspace );
        assert( (int)circbufNext + res <= circbufSize );
        circbufNext += (uint8_t)res;
        circbufLength += res;
        if( circbufLength > circbufSize )
        {
            circbufStart = circbufNext;
            circbufLength = circbufSize;
        }
        if( res < endspace )
            break;
    }
}
