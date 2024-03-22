/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2004, 2005, 2008 - Jens Wilhelm Wulf (original author)
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
#ifndef __LOGGERREADER_TTYS_H
#define __LOGGERREADER_TTYS_H

#include "LoggerReader_byte.h"

#ifdef linux
# include <termios.h> // seriell
#endif

/**
 * TODO: -eine Methode, über die man die verfügbaren Baudraten
 *        erfragen kann
 */
class LoggerReader_ttyS : public LoggerReader_byte
{
  public:
   LoggerReader_ttyS(unsigned int uBufSize,
                     const char*  sd,
                     int          baudrate);
   ~LoggerReader_ttyS();

   int  fetchData();
      
  private:
#ifdef linux
   int               serPort;          // der file-descriptor
   struct termios    saved_attributes; // Use this variable to remember original terminal attributes.
#endif
   
   void reset_input_mode();
   void set_input_mode(int baudrate);
};

#endif
