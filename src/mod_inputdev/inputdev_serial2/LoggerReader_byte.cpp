// -*- mode: c; mode: fold -*-
/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2004, 2008 - Jens Wilhelm Wulf (original author)
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

#include "LoggerReader_byte.h"
#include <stdlib.h>

// 1: ?
// 2: ?
// 3: zeigt (un)erkannte Frametypen an
#define SHOWREC 0

#define DEBUG   0

#if (SHOWREC != 0 || DEBUG != 0)
# include <stdio.h>   // printf
#endif


     LoggerReader_byte::LoggerReader_byte(unsigned int uBufSizei)
                                                                  /*{{{*/
{
#if DEBUG == 1
  printf("LoggerReader_byte::LoggerReader_byte(DaLoConf*, %i)\n", uBufSizei);
#endif
  
  fOK  = true;
  
  {
    readConf();
#if DEBUG == 1
    printf("nach readconf\n");
#endif
    inBufSize = nMaxFrameLen + 1 + 1; // Daten + Format
  }
  
  outBufSize        = uBufSizei;
  outBuf            = (unsigned char*)malloc(outBufSize);
  outBufFrames      = 0;
  outBufWr          = 0;
  outBufRd          = 0;
    
  inBuf             = (unsigned char*)malloc(inBufSize);
  inBufCntAndStatus = -1;      
  fCtrl             = false;
}
/*}}}*/

     LoggerReader_byte::~LoggerReader_byte()                      /*{{{*/
{
#if DEBUG != 0  
  printf("LoggerReader_byte::~LoggerReader_byte()\n");
#endif  
  free(inBuf);
  free(outBuf);
}
/*}}}*/



const unsigned char*  LoggerReader_byte::getFrameWithLen()        /*{{{*/
{
  return(&outBuf[outBufRd]);
}
/*}}}*/

      unsigned char*  LoggerReader_byte::getModFrameWithLen()     /*{{{*/
{
  return(&outBuf[outBufRd]);
}
/*}}}*/

const unsigned char*  LoggerReader_byte::getFrame()               /*{{{*/
{
  return(&outBuf[outBufRd+1]);
}
/*}}}*/

void LoggerReader_byte::nextFrame()                               /*{{{*/
{
  if (--outBufFrames == 0)
  {
    // Der Ausgabepuffer wurde komplett geleert. Schön, denn jetzt
    // können beide Zeiger zurückgesetzt werden.
    outBufWr = 0;
    outBufRd = 0;
  }
  else
  {
    // outBufRd muss auf den folgenden Framebeginn weitergesetzt werden.
    outBufRd += outBuf[outBufRd];
  }
}
/*}}}*/

int  LoggerReader_byte::hasFrames()                               /*{{{*/
{
  return(outBufFrames);
}
/*}}}*/

int  LoggerReader_byte::putChar(unsigned char input)              /*{{{*/
{
#if SHOWREC == 2
  printf(" %i", input);
#endif
  
  if (input == ctrlChar && fCtrl == false)
    fCtrl = true;
  else // (input != ctrlChar || fCtrl == true)
  {
    if (fCtrl)
    {
      switch (input)
      {
       case cc_bb:
#if SHOWREC == 1
        printf("\nBB");
#endif
        if (inBufCntAndStatus != -1)
        {
          // Ich war nicht idle, das ist also ein Fehler, was da gerade kam
          recErr(1, 0);
        }
        inBufCntAndStatus = -2;
        break;
        
       case cc_be:
#if SHOWREC == 1
        printf(" BE");
#endif
        // Jetzt sollte das letzte Zeichen des Frames schon empfangen sein,
        // ansonsten war das Frame kürzer als erwartet.
        if (inBufCntAndStatus != 0)
        {
#if SHOWREC == 1
          printf(" inBufCntAndStatus=%i", inBufCntAndStatus);
#endif
          recErr(1, 1);
          inBufCntAndStatus = -1; // wieder idle gehen            
        }
        else if (crcReg != 0)
        {
          // CRC-Fehler                     
          recErr(1, 2);
          inBufCntAndStatus = -1; // wieder idle gehen
        }
        else
        {
#if SHOWREC == 1
          printf(" Frame korrekt empfangen: ");
          for (int n=0; n<inBufFrameLen; n++)
          {
            printf(" %u", (unsigned int)(inBuf[n]));
          }
#endif  
          // War es ein normales Frame oder das spezielle,
          // welches den Formattyp ansagen möchte?
          if (inBuf[inBufFrameLen-1] == DaLoConst::ID_descr)
          {
            inBufCntAndStatus = -1; // wieder idle gehen
          }
          else
          {
            // es war ein normales Frame
            recFrame();
            inBufCntAndStatus = -1; // wieder idle gehen
          }
        }
          
        break;
        
       case cc_err:
#if SHOWREC == 1
        printf(" !ERR!");
#endif
        // Wenn ich idle war, ist das ok, dann muss ich nur den
        // gemeldeten Fehler weiterreichen, ansonsten habe ich mich verzettelt
        if (inBufCntAndStatus == -1)        
          recErr(0,0);
        else        
          recErr(1,4);
        
        inBufCntAndStatus = -1; // wieder idle gehen            
        break;
        
       case ctrlChar:
        // es ist ein normales Zeichen
        fCtrl = false;
        break;
          
       default:
#if SHOWREC == 1
        printf(" ?%i?", (int)(input));
#endif
        // Ein unerwartetes Zeichen nach dem Steuerzeichen
        recErr(1, 5);
        inBufCntAndStatus = -1; // wieder idle gehen            
        break;
      }
    }
      
    // --- Ein normales Zeichen im Datenstrom? ------------
    if (fCtrl == false)
    {
#if SHOWREC == 1
      printf(" %i", int(input));
#endif
      if (inBufCntAndStatus == -2)
      {
        // Das gerade empfangene Zeichen sollte der Frametyp sein.
        // Kenne ich den Frametyp, wie sollte dessen Länge sein?
        unsigned int uSize = frames.size();
        unsigned int uCnt;
        
        for (uCnt=0; uCnt < uSize && inBufCntAndStatus == -2; uCnt++)
        {
          if (input == frames[uCnt].start)
          {
            inBufFrameLen              = frames[uCnt].len+2;
            inBufCntAndStatus          = inBufFrameLen-1;
            inBuf[inBufCntAndStatus]   = frames[uCnt].ID;
            crcReg                     = input;
          }
        }
        
        // Habe ich diesen Frametyp erkannt?
        if (inBufCntAndStatus == -2)
        {
          recErr(2, input);
          inBufCntAndStatus = -1; // wieder idle gehen
#if SHOWREC == 3
          printf("Frametyp unbekannt: %i\n", input);
#endif
        }
#if SHOWREC == 3        
        else
        {
          printf("Frametyp bekannt: %i, %i\n", input, inBufFrameLen);
        }
#endif        
        
      }
      // Es sollte ein normales Zeichen innerhalb des Frames sein.
      // inBufCntAndStatus zeigt auf das als letztes beschriebene Zeichen im Puffer.
      // Wenn das schon null ist, war dieses Frame länger als erwartet
      else if (inBufCntAndStatus == 0)
      {
#if SHOWREC == 1
        printf(" !Frame zu lang!");
#endif
        recErr(1, 6);
        inBufCntAndStatus = -1; // wieder idle gehen
      }
      else if (inBufCntAndStatus > 0)
      {
        inBuf[--inBufCntAndStatus] = input;
        crcByteSchritt(input);
      }
    }
    
    // Das Steuerzeichen wurde ausgewertet
    fCtrl = false;      
  }
  
  return(0);
}
/*}}}*/

void LoggerReader_byte::recErr(unsigned char byte0, 
                               unsigned char byte1)               /*{{{*/
{
  if (outBufWr + 4 >= outBufSize)
  {
    // Pufferüberlauf
    setErr("Buffer overflow");
  }
  else
  {
#if SHOWREC == 2
    printf(" ErrFrame: %i %i\n", byte0, byte1);
#endif
    outBuf[outBufWr++] = 4;
    outBuf[outBufWr++] = DaLoConst::ID_err;
    outBuf[outBufWr++] = byte0;
    outBuf[outBufWr++] = byte1;
    outBufFrames++;
  }
}
/*}}}*/

void LoggerReader_byte::recFrame()                                /*{{{*/
{
  if (outBufWr + inBufFrameLen >= outBufSize)
  {
    // Pufferüberlauf
    setErr("Buffer overflow");
  }
  else
  {
    outBuf[outBufWr++] = inBufFrameLen;
    
#if DEBUG == 1  
    printf(" ->");
#endif  
#if SHOWREC == 2
    printf(" Frame:");
#endif
    for (int n=inBufFrameLen-1; n>0; n--)
    {
#if ((DEBUG == 1) || (SHOWREC == 2))
      printf(" %i", inBuf[n]);
#endif
      outBuf[outBufWr++] = inBuf[n];
    }
#if ((DEBUG == 1) || (SHOWREC == 2))
    printf("\n");
#endif  
    
    if (inBuf[inBufFrameLen-1] != DaLoConst::ID_ts)
      outBufFrames++;    
  }
}
/*}}}*/

int  LoggerReader_byte::addFrame(unsigned char ID,
                                 unsigned char start,
                                 int           len)               /*{{{*/
{
  T_Frame tmp;
  tmp.ID    = ID;
  tmp.start = start;
  tmp.len   = len;
  frames.push_back(tmp);
  
  if (tmp.len > nMaxFrameLen)
    nMaxFrameLen = tmp.len;
  
  return(0);
}
/*}}}*/


int  LoggerReader_byte::readConf()                                /*{{{*/
{
#if DEBUG == 1
  printf("int LoggerReader_byte::readConf()\n");
#endif
    
  // --- Zunächst alte Daten löschen ----------------------
  nMaxFrameLen = 0;
      
  // --- Die Beschreibung in el einlesen ------------------        

  // Geberframes
  addFrame(0, 0x63, 3);
  addFrame(1, 0x64, 3);
  addFrame(2, 0x65, 3);
  addFrame(3, 0x66, 3);
  addFrame(4, 0x67, 3);
  addFrame(5, 0x68, 3);
  addFrame(6, 0x69, 3);
  addFrame(7, 0x6A, 3);
  // Parameterframe
  addFrame(8, 0x62, 3);
  
#if DEBUG == 1  
  for (unsigned int n=0; n<frames.size(); n++)
  {
    printf("Frametyp: %i %i %i\n", frames[n].ID, frames[n].start, frames[n].len);
  }
#endif  
  return(0);
}
/*}}}*/

void LoggerReader_byte::crcByteSchritt(unsigned char data)        /*{{{*/
{
  for (int i=0; i<8; i++)
    {      
      crcReg <<= 1;
      
      // das nächste Datenbit ins Register schieben
      if ((data & 0x80) != 0)
        crcReg |= 1;
      
      data <<= 1;
      
      if ((crcReg & 0x100) != 0)
        crcReg = crcReg ^ crcPoly;
    }
  
  crcReg &= 0xFF;
}
/*}}}*/

void LoggerReader_byte::setErr(std::string err)                   /*{{{*/
{
  if (fOK)
    firstErr = err;
  fOK = false;
}
/*}}}*/

