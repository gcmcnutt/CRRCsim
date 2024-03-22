/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2006, 2008 - Todd Templeton (original author)
 *   Copyright (C) 2008 - Jens Wilhelm Wulf
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
// Created 11/09/06 Todd R. Templeton <ttemplet@eecs.berkeley.edu>
// Based on fdm_larcsim.h

#ifndef FDM_DISPLAYMODE_H
# define FDM_DISPLAYMODE_H

#include "../../mod_fdm_config.h"

#if (MOD_FDM_USE_DISPLAYMODE != 0)

#include "../fdm_larcsim/fdm_larcsim.h"
#include "../../mod_chardevice/chardevice.h"

#define DATA_PACKET_SIZE 70

/**
 * 
 * Jens Wilhelm Wulf, September 2005: attempt to (re)structure and simplify LaRCSim code.
 *
 * @author Jens Wilhelm Wulf
 */
class CRRC_AirplaneSim_DisplayMode : public CRRC_AirplaneSim_Larcsim
{
   friend class ModFDMInterface;
   
  public:
  
   /**
    * Returns true if this FDM should be used.
    */
   static bool UseMe(SimpleXMLTransfer* cfg);
  
   /**
    * Used for sound calculation. It returns the prop's number of revolutions
    * per second [1/s].
    */
   virtual double getPropFreq() { return(0.0); }

   /**
    * Returns velocity relative to airmass [ft/s].
    */
   virtual double getVRelAirmass() { return(0.0); };
   
  private:
   void LoadFromXML(SimpleXMLTransfer* xml,
                    SimpleXMLTransfer* cfg);

   void update(TSimInputs* inputs,
               double      dt,
               int         multiloop);
               
   void update(float*      rawinputs,
               int         n,
               double      dt,
               int         multiloop);

   /**
    * read from file
    */
   CRRC_AirplaneSim_DisplayMode(const char* filename,
                                SimpleXMLTransfer* cfg);

   /**
    * read from xml description
    */
   CRRC_AirplaneSim_DisplayMode(SimpleXMLTransfer* xml,
                                SimpleXMLTransfer* cfg);

   virtual ~CRRC_AirplaneSim_DisplayMode();
      
   void decodePacket(char* buf);
   
   CharDeviceWrapper* io;
   char buf[DATA_PACKET_SIZE];
   int nBytes;
   bool originInitialized;
   double origin[2];
};

#endif
#endif
