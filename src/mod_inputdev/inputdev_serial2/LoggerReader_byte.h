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
#ifndef __LOGGERREADER_BYTE_H
#define __LOGGERREADER_BYTE_H

#include <vector>
#include <string>

class DaLoConst
{
  public:
   /**
    * ID von speziellen Frames
    */
   enum { ID_ts = 255, ID_err = 254, ID_descr = 253 };
   
   /**
    * Länge von speziellen Frames
    */
   enum { len_ts = 2, len_err = 2 };

   /**
    * CRC der Datei
    */
   enum { crc_poly = 0x04C11DB7 };      
};


class LoggerReader_byte
{
  public:
   LoggerReader_byte(unsigned int uBufSizei);
   
   int                  hasFrames();
   const unsigned char* getFrame();
   const unsigned char* getFrameWithLen();
         unsigned char* getModFrameWithLen();
   
   bool                 isOK()         { return(fOK); };
   std::string          getErr()       { return(firstErr); };
   
   /**
    * Wird aufgerufen, um anzuzeigen, dass man das Frame, auf das der Zeiger
    * vom letzten getFrame()-Aufruf zeigt, ausgewertet hat.
    */
   void                 nextFrame();
      
   ~LoggerReader_byte();
   
  protected:
   typedef struct {
     unsigned char  ID;
     unsigned char  start;
     int            len;     
   } T_Frame;
   
   /**
    * Die Steuerzeichen der seriellen Übertragung
    */
   enum { ctrlChar = 0x1B };
   
   /**
    * KontrollCodes der seriellen Übertragung
    */
   enum { cc_bb = 0, cc_be = 1, cc_err = 2 };
   
   /**
    * Spezielle Frames
    */
   enum { sf_descr = 0x45, sf_ts = 0x61 };
   
   /**
    * Längen spezieller Frames
    */
   enum { len_descr = 4 };

   /**
    * das CRC-Polynom
    */
   enum { crcPoly = 49 };
   
   /**
    * War das letzte empfangene Zeichen das Steuerzeichen?
    */
   bool              fCtrl;
   
   /**
    * Der Puffer für die gerade einlaufenden Daten
    */
   unsigned char*    inBuf;
   
   /**
    * aktuelle Größe des obigen Puffers
    */
   int               inBufSize;
   
   /**
    * Index für obigen Puffer und Status
    * -1: idle, wartend auf Startbedingung (ctrlChar und cc_bb)
    * -2: Startbedingung ist eingelaufen, jetzt sollte der Frametyp kommen
    */
   int               inBufCntAndStatus;
   
   /**
    * Länge des gerade einlaufenden Frames
    */
   int               inBufFrameLen;
   
   /**
    * CRC-Register
    */
   int               crcReg;
   
   /**
    * Puffer für Ausgangsdaten
    */
   unsigned char*    outBuf;
   
   /**
    * Größe des obigen Puffers
    */
   unsigned int      outBufSize;

   /**
    * Anzahl der unabgeholten Frames im Ausgangspuffer
    */
   unsigned int      outBufFrames;
   
   /**
    * Schreibindex Ausgangspuffer
    */
   unsigned int      outBufWr;
   
   /**
    * Leseindex Ausgangspuffer
    */
   unsigned int      outBufRd;
   
   std::vector<T_Frame>       frames;
   int                        nMaxFrameLen;
   
   int  putChar(unsigned char ch);
   void crcByteSchritt(unsigned char data);
   void recFrame();
   void recErr(unsigned char byte0, unsigned char byte1);
   int  readConf();
   int  addFrame(unsigned char ID,
                 unsigned char start,
                 int           len);
   
   /** Die folgenden Dinge gehören eigtl. zu LoggerReader, habe ich zur 
    * Vereinfachung übernommen */
   bool fOK;
   std::string firstErr;
   void setErr(std::string err);      
};
#endif
