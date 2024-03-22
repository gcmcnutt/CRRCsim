/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005-2010 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2006, 2008 - Jan Reucker
 *   Copyright (C) 2006 - Todd Templeton
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
#ifndef CRRC_FDM_H
# define CRRC_FDM_H

#include <string>
#include <iostream>
#include <fstream>

#include "fdm_inputs.h"
#include "../mod_misc/SimpleXMLTransfer.h"
#include "../mod_math/vector3.h"
#include "../mod_math/matrix33.h"
#include "fdm_env.h"

#define FDM_LOG             0
#define FDM_LOG_POS         1
#define FDM_LOG_WIND_IN     0
#define FDM_LOG_AERO_OUT    0

/* Forward declaration to avoid pulling in crrc_animation.h */
class CRRCAnimation;

/**
 * Base class for handling the physics of an airplane.
 * You should not call any of its methods (with the exception of get-Methods). 
 * ModFDMInterface does it for you.
 * 
 * @author Jens Wilhelm Wulf
 */
class FDMBase
{
   friend class ModFDMInterface;
  public:
 
   /**
    * The world coordinate vector vWorld is transformed
    * to body coordinates and returned.
    */
   virtual CRRCMath::Vector3 WorldToBody(CRRCMath::Vector3 vWorld)=0;
  
   virtual CRRCMath::Vector3 getPos()=0;
   
   virtual double getPhi()      = 0;
   virtual double getTheta()    = 0;
   virtual double getPsi()      = 0;
   virtual double getFlightCL() = 0;
   virtual bool   isStalling()  = 0;

   /**
    * returns velocity w.r.t. earth surface
    */
   virtual CRRCMath::Vector3 getVel()   { return(CRRCMath::Vector3()); };
   virtual CRRCMath::Vector3 getAccel() { return(CRRCMath::Vector3()); };
   
   virtual CRRCMath::Vector3 getPQR()   { return(CRRCMath::Vector3()); };

   virtual double getLon() { return(-1.0); };
   virtual double getLat() { return(-1.0); };
   virtual double getAlt() { return(-1.0); };
   
   FDMBase(const char* logfilename, FDMEnviroment* myEnv);
   
   virtual ~FDMBase();
   
   /**
    * Used for sound calculation. It returns the prop's number of revolutions
    * per second [1/s].
    */
   virtual double getPropFreq() = 0;
   
   /**
    * Returns velocity relative to airmass [ft/s].
    */
   virtual double getVRelAirmass() = 0;
   
   /**
    * Returns relative battery capacity/fuel left (0..1).
    */
   virtual double getBatCapLeft() { return(0.68); };
   
   /**
    * Wingspan of the aircraft in feet
    */
   virtual double getWingspan() = 0;
   
   /**
    * computed velocity for trimmed flight in dead air
    */
   virtual double getTrimmedFlightVelocity() = 0;
   
   /**
    * returns Z coordinate of lowest point
    */
   virtual double getZLow() = 0;

   /**
    * This does only reload airplane parameters, but does not change states.
    * An FDM doesn't need to implement this method.
    * 
    * returns 0 if nothing happened, 1 on success, throws an XMLException otherwise.
    */
   virtual int ReloadParams(SimpleXMLTransfer* xml,
                            SimpleXMLTransfer* cfg) { return(0); };

   FDMEnviroment* GetEnv() { return(env); };
  
  private:
   
   virtual void update(TSimInputs* inputs,
                       double      dt,
                       int         multiloop) = 0;
   
   virtual void initAirplaneState(double dRelVel,
                                  double dPhi,
                                  double dTheta,
                                  double dPsi,
                                  double X,
                                  double Y,
                                  double Z,
                                  double R_X = 0.0,
                                  double R_Y = 0.0,
                                  double R_Z = 0.0) = 0;

  protected:
  
  /**
   * This is where I get information from the outside
   * and where log messages may be given to
   */
  FDMEnviroment* env;

  /**
   * This buffer is used to prevent creating one on every call of update().
   */
  TSimInputs myInputs;
  
#if FDM_LOG != 0
   std::ofstream logfile;
   
   int nLogCnt;
   
   int nStep;
#endif
   
   /**
    * Inline function which becomes nothing without logging.
    */
   void logNewline() 
   {
#if FDM_LOG != 0
     if (nLogCnt != 0)
       logfile << "\n";
     
     logfile << nLogCnt++ << " ";
#endif
   };
   
   /**
    * Inline function which becomes nothing without logging.
    */
   void logVal(double val)
   {
#if FDM_LOG != 0
     logfile << val << " ";
#endif
   };
      
   /**
    * Inline function which becomes nothing without logging.
    */
   void logVal(CRRCMath::Vector3 val)
   {
#if FDM_LOG != 0
     if (fabs(val.r[0]) > 2E6)
       val.r[0] = 0;
     if (fabs(val.r[1]) > 2E6)
       val.r[1] = 0;
     if (fabs(val.r[2]) > 2E6)
       val.r[2] = 0;
     
     logfile << val.r[0] << " " << val.r[1] << " " << val.r[2] << " ";
#endif
   };
      
   /**
    * Inline function which becomes nothing without logging.
    */
   void logVal(CRRCMath::Matrix33 const& val)
   {
#if FDM_LOG != 0
     
     for (unsigned int m=0; m<3; m++)
       for (unsigned int n=0; n<3; n++)
       {
         if (fabs(val.v[m][n]) > 2E6)
         {
           logfile << val.v[m][n] << " ";
//           logfile << "0 ";
         }
         else
           logfile << val.v[m][n] << " ";
       }
#endif
   };
      
};

/**
 * The purpose of this class is to hide the simulation core from the rest of
 * the application and to provide a lean interface. This makes it easier to 
 * use different models (beside larcsim).
 *
 * @author Jens Wilhelm Wulf
 */
class ModFDMInterface
{
  public:
   
   ModFDMInterface();
   
   virtual ~ModFDMInterface();
   
   virtual void Clean();  
  
   /**
    * Loads an airplane with fdm_testmode, which is a special FDM only
    * mapping control inputs to simple changes of position/orientation.
    * 
    * Parameters are position coordinates in feet.
    */
   void loadAirplaneTestmode(double dNorth, double dEast, double dDown);
   
   /**
    * Load airplane description from file. This method should be able to handle
    * different filetypes (create an instance of the correct FDMBase).
    */
   virtual void loadAirplane(const char* filename, 
                             FDMEnviroment* myEnv,
                             SimpleXMLTransfer* cfg);

   /**
    * Load airplane from xml description. This method should be able to handle
    * different filetypes (create an instance of the correct FDMBase).
    */
   void loadAirplane(SimpleXMLTransfer* xml,
                     FDMEnviroment* myEnv,
                     SimpleXMLTransfer* cfg);

   /**
    * Init state of airplane (after it has been loaded from a file using loadAirplane).
    * 
    * @param dRelVel velocity (relative to trimmed flight)
    */
   void initAirplaneState(double dRelVel,
                          double dPhi,
                          double dTheta,
                          double dPsi,
                          double X,
                          double Y,
                          double Z,
                          double R_X = 0.0,
                          double R_Y = 0.0,
                          double R_Z = 0.0);
   
   /**
    * Update timestep.
    */
   void update(TSimInputs* inputs,
               double      dt,
               int         multiloop);

   /**
    * Return the launch presets
    */
   SimpleXMLTransfer* getLaunchPresets() {return launch_presets;};

   /**
    * Return the mixer presets
    */
   SimpleXMLTransfer* getMixerPresets() {return mixer_presets;};

   /**
    * Pointer to FDM in use.
    */
   FDMBase* fdm;
  
   /**
    * This only tries to reload airplane parameters, but does not change states.
    * 
    * returns 0 if nothing happened, 1 on success, throws an XMLException otherwise.
    */
   int ReloadParams(SimpleXMLTransfer* xml,
                    SimpleXMLTransfer* cfg);
       
  protected:
   /**
    * Pointer to airplane-specific launch presets
    */
   SimpleXMLTransfer* launch_presets;

   /**
    * Pointer to airplane-specific mixer presets
    */
   SimpleXMLTransfer* mixer_presets;
};

#endif
