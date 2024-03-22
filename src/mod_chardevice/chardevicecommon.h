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
// chardevicecommon.h
//
//    Common includes and definitions that are used by chardevice implementation
//    files.
//
// authors
//
//    Todd R. Templeton <ttemplet@eecs.berkeley.edu>
//
//----------------------------------------------------------------------------------


#ifndef __CHARDEVICECOMMON_H__
#define __CHARDEVICECOMMON_H__

#include <stdlib.h>
#include <stdio.h>

#ifdef WIN32
#include <Windows.h>
#include <winsock.h>
#include <sys/timeb.h>
#include <time.h>
#elif defined(NBMINGW)
#include <basictypes.h>
#include <serial.h>
#include <startnet.h>
#include <ip.h>
#include <tcp.h>
#include <udp.h>
#else // ifdef WIN32
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include <netinet/tcp.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#ifndef SOL_TCP
/* for ifndef SOL_TCP below */
#include <netdb.h>
#endif
#endif // ifdef WIN32

#include "chardevicebase.h"


#ifdef WIN32
typedef SOCKADDR sockaddr;
typedef int socklen_t;
#else
#define SOCKET_ERROR -1
#ifndef SOL_TCP
#define SOL_TCP getprotobyname("TCP")->p_proto
#endif
#ifndef MSG_NOSIGNAL
/* FreeBSD and OS X use SO_NOSIGPIPE instead */
#define MSG_NOSIGNAL SO_NOSIGPIPE
#endif
#ifndef ENONET
/* always use these two together */
#define ENONET EHOSTDOWN
#endif
/* ensure that either both or neither of {VSWTC,VSWTCH} are set */
#if defined(VSWTCH) && !defined(VSWTC)
#define VSWTC VSWTCH
#endif
#if defined(VSWTC) && !defined(VSWTCH)
#define VSWTCH VSWTC
#endif
#endif
/* Windows and NetBurner uses char* for read()/recv() and write()/send() */
#ifdef WIN32
typedef char* read_write_type;
#elif defined(NBMINGW)
typedef char* read_write_type;
#else
typedef void* read_write_type;
#endif


#endif // __CHARDEVICECOMMON_H__
