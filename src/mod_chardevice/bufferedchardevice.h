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
// bufferedchardevice.h
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


#ifndef __BUFFEREDCHARDEVICE_H__
#define __BUFFEREDCHARDEVICE_H__

#include "chardevicebase.h"


// circular buffer over any character device; should be subclassed
class BufferedCharDevice
{
private:
    
    // whether to delete charDevice on cleanup
    bool deleteCharDevice;
    
    
protected:
    
    // char device in use
    CharDevice *charDevice;
    
    // circular buffer
    int circbufSize;
    char *circbuf;
    uint8_t circbufStart;
    uint8_t circbufNext;
    int circbufLength;
    
    
protected:

    // initialization (device = device options string, wait = whether to wait for connection/reconnection)
    void init( const char *device, bool wait = true );
    // initialization (charDevice_ = CharDevice to use)
    void init( CharDevice *_charDevice );
    // cleanup
    void cleanup( void );
    
    // read bytes into circular buffer (uses charDevices's maxInterval and wait)
    void read_into_buffer( void );
    
    
public:

    // constructor
    BufferedCharDevice( ) : charDevice( 0 ), circbuf( 0 )
    {
    }
    
    
    // destructor
    virtual ~BufferedCharDevice( )
    {
    }
};


#endif // __BUFFEREDCHARDEVICE_H__
