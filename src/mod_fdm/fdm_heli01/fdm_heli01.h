// -*- mode: c; mode: fold -*-
/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2008, 2009 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2008 - Jan Reucker
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
#ifndef FDM_HELI01_H
# define FDM_HELI01_H

# include <stdexcept>
# include <vector>
# include "../ls_types.h"
# include "../eom01/eom01.h"
# include "../../mod_math/vector3.h"
# include "../../mod_math/matrix33.h"
# include "../power/power.h"
# include "../../mod_misc/crrc_rand.h"
# include "../gear01/gear.h"

/**
 * simple physical model for a helicopter
 * 
 * @author Jens Wilhelm Wulf
 */
class CRRC_AirplaneSim_Heli01 : public EOM01
{
   friend class ModFDMInterface;
   friend class CRRC_AirplaneSim_DisplayMode;
   
  public:

   virtual bool   isStalling() { return(false); };

   virtual double getFlightCL() { return(0.0); };
   
   /**
    * Used for sound calculation. It returns the prop's number of revolutions
    * per second [1/s].
    */
   virtual double getPropFreq();

   /**
    * Returns relative battery capacity/fuel left (0..1).
    */
   virtual double getBatCapLeft() { return(power->getBatteryMin()); };

   /**
    * computed velocity for trimmed flight in dead air [ft/s]
    */
   virtual double getTrimmedFlightVelocity() { return(0); };

   /**
    * Wingspan of the aircraft in feet
    */
   virtual double getWingspan() { return(wheels.getWingspan()); };
   
   /**
    * returns Z coordinate of lowest point
    */
   virtual double getZLow() { return(wheels.getZLow()); };

   /**
    * This only tries to reload airplane parameters, but does not change states.
    */
   virtual int ReloadParams(SimpleXMLTransfer* xml, 
                            SimpleXMLTransfer* cfg);
  private:

   void LoadFromXML(SimpleXMLTransfer* xml, int nVerbosity);
  
   void InitStates(); 
  
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
    * read from file
    */
   CRRC_AirplaneSim_Heli01(const char* filename, FDMEnviroment* myEnv, SimpleXMLTransfer* cfg);

   /**
    * read from xml description
    */
   CRRC_AirplaneSim_Heli01(SimpleXMLTransfer* xml, FDMEnviroment* myEnv, SimpleXMLTransfer* cfg);

   virtual ~CRRC_AirplaneSim_Heli01();
   
  private:
  
   /// @name Gear and ground interaction
   //@{

   /**
    * Vector containing all hard points/wheels
    */
   WheelSystem wheels;
   //@}

   
   
  private:
   
   void gear(TSimInputs* inputs, CRRCMath::Vector3& v_F, CRRCMath::Vector3& v_M);
   void aero(double dt, TSimInputs* inputs, CRRCMath::Vector3& v_F, CRRCMath::Vector3& v_M);
   void engine(SCALAR dt, TSimInputs* inputs, CRRCMath::Vector3& v_F, CRRCMath::Vector3& v_M);
   virtual void ls_step_init();
 
   float GroundEffect(float dRotorToGround);

 private:   
  
   /**
    * Resulting torque is multiplied element-wise with this vector.
    * This vector is (1|1|1), except when adjusting parameters.
    */
   CRRCMath::Vector3 MulM;
  
   /**
    * Resulting force is multiplied element-wise with this vector.
    * This vector is (1|1|1), except when adjusting parameters.
    */
   CRRCMath::Vector3 MulF;  
  
   /**
    * activate fixed horizon controller
    */
   bool fFixedHorizon;
  
   /// @name written by constructor
   //@{
   
   /**
    * Propulsion system: batteries, shafts, engines, propellers.
    */
   Power::Power* power;
   
   /**
    * Highest value of a hard points y-coordinate -- it is assumed 
    * to be the rotor radius. If there's a parameter for rotor
    * radius one day, we'll use that.
    */
   double dRotorRadius;
  
   /**
    * Lowest value of a hard points z-coordinate -- it is assumed 
    * to be the main rotor coordinate.
    */
   double dRotorZ;
  

   //@}

  /// @aero
  //@{
  double yaw_ctrl;
  double yaw_off;
  double yaw_damp;
  double yaw_damp_min_rel;
  double roll_ctrl;
  double roll_damp;
  double pitch_ctrl;
  double pitch_damp;
  double cp_ctrl;
  double cp_off;
  bool   fFixedPitch;
  double speed_damp;
  
  double yaw_dist;
  double roll_dist;
  double pitch_dist;
  RandGauss rnd_yaw;
  RandGauss rnd_roll;
  RandGauss rnd_pitch;
  CRRCMath::PT1 filt_rnd_yaw;
  CRRCMath::PT1 filt_rnd_roll;
  CRRCMath::PT1 filt_rnd_pitch;
  double in_rnd_yaw;
  double in_rnd_roll;
  double in_rnd_pitch;
  double dist_t;
  double dist_t_init;
  
  double dHeadingHold;
  double dHeadingHoldInt;
  
  bool fCoaxial;

  double dForwardToRoll;
  
  double yaw_moment_mul;
  double yaw_vane;  
  double pitch_vane;
  double cp_to_yaw;  
  
   /**
    * Ground effect parameters, see code
    */
  double dGEDistMul;
  
  //@}  
  
};

#endif
