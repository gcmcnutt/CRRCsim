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
// chardevice.h
//
//    Cross-platform one-to-one communication over serial, TCP (client and server),
//    and UDP (client and server) through a simple read/write interface that is
//    robust to disconnection. Also a "wrapper" char device that allows any one of
//    the above to be used transparently, and a "buffered character device" that
//    serves as a base class for message-passing architectures that receive a single
//    type of message that is no more than 256 characters in length (transparently
//    over any one of the supported interfaces).
//
// authors
//
//    Todd R. Templeton <ttemplet@eecs.berkeley.edu>
//    David H. Shim <hcshim@eecs.berkeley.edu> (QNX serial initialization)
//
//----------------------------------------------------------------------------------


//----------------------------------------------------------------------------------
//
// Status
//
// Linux, g++: Tested and working
// Linux PPC, g++: TCP/UDP tested and working, serial untested
// Windows XP, MS Visual C++: Tested and working
// Windows XP, MinGW g++: Tested and working
// OS X, g++: Compiles, otherwise untested
// QNX: Serial initialization routine included, otherwise untested
//
//----------------------------------------------------------------------------------


//----------------------------------------------------------------------------------
//
// Options String Format
//
// SerialCharDevice: "device,speed", e.g. "/dev/ttyS0,115200"
// TCPCharDevice: "host,port", e.g. "192.168.0.5,12345"
// TCPServerCharDevice: "host,port" or "host/netmask,port", e.g. "192.168.0.5,12345"
//   or "192.168.0.5/255.255.255.0,12345"
// UDPCharDevice: "host,port", e.g. "192.168.0.5,12345"
// UDPServerCharDevice: "host,port" or "host/netmask,port", e.g. "192.168.0.5,12345"
//   or "192.168.0.5/255.255.255.0,12345"
// FileCharDevice: "infile[,outfile]", e.g. "infile.txt,outfile.txt" or "infile.txt"
// CharDeviceWrapper: any of the above, with "serial", "tcp", "tcpserver", "udp",
//   "udpserver", or "file" in front, e.g. "serial,/dev/ttyS0,115200" or
//   "udp,192.168.0.5,12345"
//
//----------------------------------------------------------------------------------


#ifdef __cplusplus


#ifndef __CHARDEVICE_CPP_H__
#define __CHARDEVICE_CPP_H__


#include "chardevicebase.h"
#include "serialchardevice.h"
#include "socketchardevicebase.h"
#include "tcpchardevice.h"
#include "tcpserverchardevice.h"
#include "udpchardevice.h"
#include "udpserverchardevice.h"
#include "filechardevice.h"
#include "chardevicewrapper.h"
#include "bufferedchardevice.h"


#endif // __CHARDEVICE_CPP_H__


#else // __cplusplus


#ifndef __CHARDEVICE_C_H__
#define __CHARDEVICE_C_H__


#include "chardevicec.h"


#endif // __CHARDEVICE_C_H__


#endif // __cplusplus
