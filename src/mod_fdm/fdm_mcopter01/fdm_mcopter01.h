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
#ifndef FDM_MCOPTER01_H
# define FDM_MCOPTER01_H

# include <stdexcept>
# include <vector>
# include "../ls_types.h"
# include "../eom01/eom01.h"
# include "../../mod_math/vector3.h"
# include "../../mod_math/matrix33.h"
# include "../power/power.h"
# include "../../mod_misc/crrc_rand.h"
# include "../gear01/gear.h"
# include "../../mod_cntrl/controller.h"


class Propdata
{
public:
  Propdata(SimpleXMLTransfer* cfg);
  double x;  
  double y;  
  double mul_r;  
};

/**
 * Simple physical model for a multicopter (quadrocopter, hexacopter or whatever).
 * 
 * What it currently lacks:
 *   - reduced efficiency of coaxial rotors is not taken care of
 *   - some real problems are not modeled (gyro drift) and it is assumed
 *     that it knows its attitude exactly
 *
 * This model can be used with a controller 'Cntrl_Omega'.
 * 
 * @author Jens Wilhelm Wulf
 */
class CRRC_AirplaneSim_MCopter01 : public EOM01
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
  virtual double getBatCapLeft() { return(power[0]->getBatteryMin()); };
  
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
  CRRC_AirplaneSim_MCopter01(const char* filename, FDMEnviroment* myEnv, SimpleXMLTransfer* cfg);
  
  /**
   * read from xml description
   */
  CRRC_AirplaneSim_MCopter01(SimpleXMLTransfer* xml, FDMEnviroment* myEnv, SimpleXMLTransfer* cfg);
  
  virtual ~CRRC_AirplaneSim_MCopter01();
  
private:
  
  /**
   * Vector containing all hard points/wheels
   */
  WheelSystem wheels;
  
private:
  
  void gear(TSimInputs* inputs, CRRCMath::Vector3& v_F, CRRCMath::Vector3& v_M);
  void aero(double dt, CRRCMath::Vector3& v_F, CRRCMath::Vector3& v_M);
  void engine(SCALAR             dt, 
              TSimInputs*        inputs,
              CRRCMath::Vector3  v_URel,
              CRRCMath::Vector3& v_F,
              CRRCMath::Vector3& v_M);
  virtual void ls_step_init();
  
  float GroundEffect(float dRotorToGround);
  
private:   
  
  /// @name written by constructor
  //@{
  
  /**
   * Propulsion system: batteries, shafts, engines, propellers.
   */
  std::vector<Power::Power*> power;
  
  std::vector<Propdata> props;
  
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
  double speed_damp;
  
  double yaw_damp1;
  double roll_damp1;
  double yaw_damp2;
  double roll_damp2;
  
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
    
  /**
   * Ground effect parameters, see code
   */
  double dGEDistMul;
  
  //@}  

  double dURef;
  
  /**
   * List of active controllers
   */
  std::vector<Controller*> controllers;
  
};

#endif
