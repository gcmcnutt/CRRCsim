// -*- mode: c; mode: fold -*-
/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2006, 2008 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2006 - Jan Reucker
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
//
#ifndef FDM_TESTMODE_H
# define FDM_TESTMODE_H

# include <stdexcept>
# include "../ls_types.h"
# include "../fdm.h"
# include "../../mod_math/vector3.h"

/**
 *
 * This is not a real flight dynamics model. It only shows simple reactions 
 * to inputs in order to check whether input channels are connected as intended.
 *
 * @author Jens Wilhelm Wulf
 */
class CRRC_AirplaneSim_TestMode : public FDMBase /*{{{*/
{
   friend class ModFDMInterface;
   
  public:
  
   // only a dummy here...
   virtual CRRCMath::Vector3 WorldToBody(CRRCMath::Vector3 vWorld) { return vWorld; };

   virtual bool   isStalling() { return(0); };
   virtual CRRCMath::Vector3 getPos();   
   virtual double getFlightCL() { return(0.0); };
   virtual double getPhi();
   virtual double getTheta();
   virtual double getPsi();

   /**
    * Used for sound calculation. It returns the prop's number of revolutions
    * per second [1/s].
    * jwtodo: simply return something related to throttle input
    */
   double getPropFreq() { return(0); };

   /**
    * Returns velocity relative to airmass [ft/s].
    */
   virtual double getVRelAirmass() { return(0); };

   /**
    * computed velocity for trimmed flight in dead air
    */
   virtual double getTrimmedFlightVelocity() {return(1); };

   /**
    * returns Z coordinate of lowest point
    */
   virtual double getZLow() { return(1); };

   /**
    * Wingspan of the aircraft in feet
    */
   virtual double getWingspan() { return(1); };
   
  private:
   void update(TSimInputs* inputs,
               double      dt,
               int         multiloop);

   virtual void initAirplaneState(double dRelVel,
                                  double dPhi,
                                  double dTheta,
                                  double dPsi,
                                  double X,
                                  double Y,
                                  double Z,
                                  double R_X,
                                  double R_Y,
                                  double R_Z);
   
   /**
    * Airplane position is at ipos
    */
   CRRC_AirplaneSim_TestMode(CRRCMath::Vector3 ipos);

   virtual ~CRRC_AirplaneSim_TestMode();

  private:
   CRRCMath::Vector3 basepos;
   CRRCMath::Vector3 pos;
   CRRCMath::Vector3 angle;
   double  dPsi0;
};
/*}}}*/

#endif
