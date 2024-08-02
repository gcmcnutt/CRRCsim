/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2007 Jan Reucker (original author)
 * Copyright (C) 2008 Jens Wilhelm Wulf
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */
  

/*****************************************************************************
Title: CRRCsim, the Charles River Radio Control Club Flight Simulator Project
Authors:
Jan Kansky:  Programming
Mark Drela:  Aerodynamics

Purpose:  The idea for CRRCsim is to take away any last excuse you might
have for not using a flight simulator to keep your thumbs certified for RC
flying.   It's free, it works, enjoy.  This is an open source project, so if
you don't like the way something works, help us fix it!

Thanks:
  Bruce Jackson for the LaRCsim framework.
  Flight Gear project for the sky sphere.

Contacts:
If you'd like to help with CRRCSIM, then send me an email!
    email                : kansky@ll.mit.edu
*****************************************************************************/

/** \file crrc_system.cpp
 *
 *  This file contains some platform-specific stuff for
 *  system interaction.
 *
 *  \author Jan Reucker
 */

#include "crrc_system.h"

#ifdef WIN32
  #include <windows.h>
#else
  #include <sys/utsname.h> 
  #include <string.h>
  #include <errno.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <iostream>
#include <sstream>

// Some stuff for printing the OS version
#ifdef WIN32
typedef struct
{
  unsigned int version;
  char         name[32];
} T_VersionName;

static const T_VersionName WinVersion[] =
{
  {(4 << 8) |  0, "Windows 95"},
  {(4 << 8) | 10, "Windows 98"},
  {(4 << 8) | 90, "Windows ME"},
  {(3 << 8) | 51, "Windows NT 3.51"},
  {(4 << 8) |  0, "Windows NT 4.0"},
  {(5 << 8) |  0, "Windows 2000"},
  {(5 << 8) |  1, "Windows XP"},
  {(6 << 8) |  0, "Windows Vista"},
  {0, "unknown"}
};
#endif // WIN32

#define OSVERSIONSTRINGBUFFERLEN (128)
static char cOSVersionStringBuffer[OSVERSIONSTRINGBUFFERLEN];




/**
 *  Send a text message to the operating system, using something
 *  as close to a MessageBox as possible.
 *
 *  On Windows we'll use the MessageBox() function. On Linux
 *  we'll try to use xmessage. In any case, print the message
 *  to stdout or stderr, depending on the message box type.
 *
 *  Credits go to the Allegro developers
 *  (http://sourceforge.net/projects/alleg/), I borrowed some
 *  ideas for the different implementations from their library.
 *
 *  \param msg  The message string (must be nul-terminated)
 *  \param type Type of the message (information, warning or error (default))
 */
void SystemMessage(const char * msg, SM_TYPE type)
{
  #ifdef WIN32
  UINT icon = MB_ICONINFORMATION;
  #endif

  // In any case, print the message to stdout/stderr
  switch (type)
  {
    case SM_ERROR:
      fprintf(stderr, "System error: %s\n", msg);
      #ifdef WIN32
      icon = MB_ICONERROR;
      #endif
      break;

    case SM_INFO:
      printf("System info: %s\n", msg);
      #ifdef WIN32
      icon = MB_ICONINFORMATION;
      #endif
      break;

    case SM_WARNING:
      printf("System warning: %s\n", msg);
      #ifdef WIN32
      icon = MB_ICONWARNING;
      #endif
      break;

    default:
      fprintf(stderr, "Invalid SystemMessage type: %d\n", type);
      printf("System message: %s\n", msg);
      #ifdef WIN32
      icon = MB_ICONINFORMATION;
      #endif
      break;
  }

  // now try to open a message box, depending on the target system
  #ifdef WIN32

  MessageBox(NULL, msg, "CRRCsim system message", MB_OK | icon);

  #elif defined linux
  // char *buf = new char[64 + strlen(msg)];
  // snprintf(buf, 63 + strlen(msg), "xmessage -center -buttons OK:1 \"%s\"", msg);
  // if (system(buf))
  //   std::cerr << msg << "\n";
  // delete[] buf;

  #elif defined MACOSX

  // not implemented yet
  
  #endif
}


/**
 *  Get the name and version of the operating system we're running on
 *  as a text string.
 *
 *  \return A pointer to a static buffer containing the string
 */
char * getOSVersionString()
{
  #ifdef WIN32

  int i = 0;
  while ((WinVersion[i].version != _winver) && (WinVersion[i].version != 0))
  {
    i++;
  }
  _snprintf(cOSVersionStringBuffer, OSVERSIONSTRINGBUFFERLEN - 1,
            "Windows v%d.%d (%s)", _winmajor, _winminor, WinVersion[i].name);
  cOSVersionStringBuffer[OSVERSIONSTRINGBUFFERLEN - 1] = '\0';

  #else   // should work for Linux and OSX
  
  struct utsname *prInfo;
  prInfo = new struct utsname;
  if (uname(prInfo) == 0)
  {
    snprintf(cOSVersionStringBuffer, OSVERSIONSTRINGBUFFERLEN - 1,
            "%s %s", prInfo->sysname, prInfo->release);
  }
  else
  {
    // call to uname() failed
    perror("Unable to retrieve version information with uname()");
    #ifdef MACOSX
    strcpy(cOSVersionStringBuffer, "Mac OS X (unknown version)");
    #elif defined linux
    strcpy(cOSVersionStringBuffer, "Linux (unknown version)");
    #else
    strcpy(cOSVersionStringBuffer, "unknown operating system");
    #endif
  }
  delete prInfo;

  #endif
  
  return cOSVersionStringBuffer;
}


/**
 *  Get the current system time and date as a formatted string
 *
 *  This method will return the current date and time as a string
 *  in the format yyyy-mm-dd hh:mm:ss
 *
 *  \param t Reference to a std::string that shall be set to the
 *           system date and time
 */
void getSystemTimeString(std::string& t)
{
  time_t current_time;
  struct tm *now;
  std::ostringstream tmp;
  
  time(&current_time);
  
  if (current_time != -1)
  {
    now = localtime(&current_time);
    
    tmp << now->tm_year + 1900 << "-" << now->tm_mon + 1 << "-" << now->tm_mday;
    tmp << " " << now->tm_hour << ":" << now->tm_min << ":" << now->tm_sec;
  }
  else
  {
    tmp << "yyyy-mm-dd hh:mm:ss";
  }
  t = tmp.str();
}

