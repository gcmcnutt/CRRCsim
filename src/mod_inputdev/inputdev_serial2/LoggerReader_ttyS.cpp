// -*- mode: c; mode: fold -*-
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

#include "LoggerReader_ttyS.h"

#ifdef linux
# include <fcntl.h>   // seriell
# include <unistd.h>  // isatty
#endif

#define DEBUG   0

#if DEBUG != 0
# include <stdio.h>   // printf
#endif


     LoggerReader_ttyS::LoggerReader_ttyS(unsigned int uBufSize,
                                          const char*  sd,
                                          int          baudrate) : LoggerReader_byte(uBufSize)
                                                         /*{{{*/
{
#if DEBUG == 1
  printf("LoggerReader_ttyS::LoggerReader_ttyS(conf, %i, %s, %i)\n", uBufSize, sd, baudrate);
#endif
  
#ifdef linux
  serPort = open(sd, O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (serPort < 0)
  {
    setErr("Fehler beim Öffnen des Ports\n");
  }
  else
  {
    set_input_mode (baudrate);    
    tcflush(serPort, TCIOFLUSH);  // Ein- und Ausgangspuffer leeren (leeren, nicht schicken!!)
  }
#else
  setErr("Sorry, this code only works on Linux up to now...");
#endif  
}
/*}}}*/

     LoggerReader_ttyS::~LoggerReader_ttyS()             /*{{{*/
{
#if DEBUG == 1
  printf("LoggerReader_ttyS::~LoggerReader_ttyS()\n");
  printf("Schliesse Schnittstelle\n");
#endif  
#ifdef linux
  reset_input_mode();
  if (close(serPort))
  {
    setErr("Fehler beim Schliessen des Ports\n");
  }
#endif
}
/*}}}*/

int  LoggerReader_ttyS::fetchData()                      /*{{{*/
{
#ifdef linux
  unsigned char input;
  int           nRetcodeSlave;
 
  while ((nRetcodeSlave = ::read(serPort, &input, 1)) == 1)
  {
# if DEBUG == 1
    printf("%i\n", input);
# else
    putChar(input);
# endif
  }
#endif  
  return(0);
}
/*}}}*/

void LoggerReader_ttyS::reset_input_mode()               /*{{{*/
{
#ifdef linux
  tcsetattr (serPort, TCSANOW, &saved_attributes);
#endif
}
/*}}}*/

void LoggerReader_ttyS::set_input_mode(int baudrate)     /*{{{*/
{
#ifdef linux
  struct  termios tattr;
  speed_t speed;

  switch (baudrate)
  {
   case 0:
    speed = B50;
    break;
   case 75:
    speed = B75;
    break;
   case 110:
    speed = B110;
    break;
   case 134:
    speed = B134;
    break;
   case 150:
    speed = B150;
    break;
   case 200:
    speed = B200;
    break;
   case 300:
    speed = B300;
    break;
   case 600:
    speed = B600;
    break;
   case 1200:
    speed = B1200;
    break;
   case 1800:
    speed = B1800;
    break;
   case 2400:
    speed = B2400;
    break;
   case 4800:
    speed = B4800;
    break;
   case 9600:
    speed = B9600;
    break;
   case 19200:
    speed = B19200;
    break;
   case 38400:
    speed = B38400;
    break;
   case 57600:
    speed = B57600;
    break;
   case 115200:
    speed = B115200;
    break;
   case 230400:
    speed = B230400;
    break;
   default:
    speed = B19200;
    setErr("Illegal baudrate\n");
    break;
  }
  
  // Make sure serPort is a terminal.
  if (!isatty (serPort))
  {
    setErr("Not a terminal.\n");
  }
  
  if (fOK)
  {
    // Save the terminal attributes so we can restore them later.
    tcgetattr (serPort, &saved_attributes);

    // Set the funny terminal modes. Siehe "man termios" !!
    tcgetattr (serPort, &tattr);
    /*
     tattr.c_lflag &= ~(ICANON|ECHO); // Clear ICANON and ECHO. macht cfmakeraw !!
     tattr.c_iflag |= INPCK;          // enable input parity checking
     tattr.c_iflag &= ~IGNPAR;        // don't ignore framing errors and parity errors
     tattr.c_iflag |= PARMRK;         // ein Zeichen mit parity- oder framing-Fehler bekommt
     // \377  \0 vorangestellt.
     */

    tattr.c_iflag &= ~IXON;          // disable XON/XOFF flow control on output
    tattr.c_iflag &= ~IXOFF;         // disable XON/XOFF flow control on input
    /* tattr.c_cflag &= ~PARODD;        // parity for input and output is even.   */

    /*  tattr.c_cflag &= ~(CCTS_OFLOW | CRTS_IFLOW | MDMBUF); */
    tattr.c_cflag |= CLOCAL;
    tattr.c_cflag &= ~CRTSCTS;
    
    tattr.c_cc[VMIN] = 0;
    tattr.c_cc[VTIME] = 0;
    cfsetospeed(&tattr, speed);         // BAUD-rate einstellen
    cfsetispeed(&tattr, speed);         // BAUD-rate einstellen
    cfmakeraw(&tattr);                  // damit man Zugriff auf die "rohen" Daten bekommt
    /*  tattr.c_cflag |= PARENB;        // enable  parity  generation  on  output  and  parity
     // checking for input. cfmakeraw stellt das aus!
     */
    
    tcsetattr (serPort, TCSAFLUSH, &tattr);
  }
#endif
}
/*}}}*/
