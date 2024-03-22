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
#ifndef FDM_002_H
# define FDM_002_H

# include <stdexcept>
# include <vector>
# include "../ls_types.h"
# include "../fdm.h"

# include "../../mod_math/vector3.h"
# include "../../mod_math/matrix33.h"
# include "../power/power.h"
# include "../gear01/gear.h"
# include "../physics/eom.h"

/**
 * Jens Wilhelm Wulf, September 2005
 * 
 * This even simpler than CRRC_AirplaneSim_Larcsim, as it does model a flat earth only. 
 * We don't need more for usual model airplane use.
 *
 */  
class CRRC_AirplaneSim_002 : public FDMBase
{
   friend class ModFDMInterface;
  public:

   /**
    * The world coordinate vector vWorld is transformed
    * to body coordinates and returned.
    */
   virtual CRRCMath::Vector3 WorldToBody(CRRCMath::Vector3 vWorld) 
   { 
     return eom.conv.body(vWorld);
   };

   virtual bool   isStalling() { return(stalling != 0); };
   virtual CRRCMath::Vector3 getPos();
   virtual double getFlightCL() { return(0.0); };
   virtual double getPhi();
   virtual double getTheta();
   virtual double getPsi();

   /**
    * Used for sound calculation. It returns the prop's number of revolutions
    * per second [1/s].
    */
   virtual double getPropFreq();

   /**
    * Returns velocity relative to airmass [ft/s].
    */
   virtual double getVRelAirmass() { return(v_V_body.length()); };

   /**
    * Returns relative battery capacity/fuel left (0..1).
    */
   virtual double getBatCapLeft() { return(power->getBatteryMin()); };

   /**
    * Wingspan of the aircraft in feet
    */
   virtual double getWingspan() { return(B_ref); };
   
   /**
    * computed velocity for trimmed flight in dead air
    */
   virtual double getTrimmedFlightVelocity() {return(trimmedFlightVelocity);}; 

   /**
    * returns Z coordinate of lowest point
    */
   virtual double getZLow() { return(wheelsys.getZLow()); };

  private:
   
   void LoadFromXML(SimpleXMLTransfer* xml);
  
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
   CRRC_AirplaneSim_002(const char* filename, FDMEnviroment* myEnv);

   /**
    * read from xml description
    */
   CRRC_AirplaneSim_002(SimpleXMLTransfer* xml, FDMEnviroment* myEnv);
   
   virtual ~CRRC_AirplaneSim_002();

  private:
   
   EOM_6DOF   eom;

   /// @name Aerodynamic data
   //@{
   SCALAR  C_ref;    //  reference chord (ft)
   SCALAR  B_ref;    //  reference span  (ft)
   SCALAR  S_ref;    //  reference area (ft^2)
   SCALAR  U_ref;    //  reference speed for Re-scaling of CD_prof  (ft/s)
   
   SCALAR  Alpha_0;  //  baseline alpha (rad)
   
   SCALAR  Cm_0   ;  //  baseline Cm at Alpha_0
   SCALAR  CL_0   ;  //  baseline CL at Alpha_0   
   SCALAR  CL_max ;  //  positive stall limit
   SCALAR  CL_min ;  //  negative stall limit
   SCALAR  CD_prof;  //  profile CD at U_ref
   
   SCALAR  Uexp_CD;  //  for Re-scaling of CD_prof  ~ (U/U_ref)^Uexp_CD
   
   SCALAR  CL_a;     // lift-force   / alpha    ~  2 pi / (1 + 2/AR)
   SCALAR  Cm_a;     // pitch-moment / alpha    (pitch stability)
   SCALAR  CY_b;     // side-force  / sideslip
   SCALAR  Cl_b;     // roll-moment / sideslip (crucial for rudder-only turns)   
   SCALAR  Cn_b;     //  yaw-moment  / sideslip (yaw stability)
   
   SCALAR  CL_q;     //  lift-force   / pitch-rate
   SCALAR  Cm_q;     //  pitch-moment / pitch-rate  (pitch damping)
   SCALAR  CY_p;     //  side-force  / roll-rate
   SCALAR  Cl_p;     //  roll-moment / roll-rate   (roll damping)
   SCALAR  Cn_p;     //  yaw-moment  / roll-rate   (yaw-roll coupling)   
   SCALAR  CY_r;     //  side-force  / yaw-rate
   SCALAR  Cl_r;     //  roll-moment / yaw-rate
   SCALAR  Cn_r;     //  yaw-moment  / yaw-rate  (yaw damping)
   
   SCALAR  CL_de;    //  lift-force   / elevator
   SCALAR  Cm_de;    //  pitch-moment / elevator
   SCALAR  CY_dr;    // side-force  / rudder
   SCALAR  Cl_dr;    // roll-moment / rudder
   SCALAR  Cn_dr;    // yaw-moment  / rudder
   SCALAR  CY_da;    // side-force  / aileron
   SCALAR  Cl_da;    // roll-moment / aileron
   SCALAR  Cn_da;    // yaw-moment  / aileron
   
   SCALAR eta_loc;
   SCALAR CG_arm;
   SCALAR CL_drop;
   SCALAR CD_stall;
   
   SCALAR span_eff;  // span efficiency: Effective span  0.95 for most planes, 0.85 flying wing
   SCALAR CL_CD0;    // CL at minimum profile CD: 0.30 for 7037, 0.15 MH32, 0.0 RG15, AGxx, power
   SCALAR CD_CLsq;   // d(CD)/d(CL^2),  curvature of parabolic profile polar: 0.01 composites, 0.015 saggy ships, 0.02 beat up ship
   SCALAR CD_AIsq;   // d(CD)/d(aileron^2) , curvature of ail. CD influence: 0.01/(max_aileron)^2
   SCALAR CD_ELsq;   // d(CD)/d(elevator^2), curvature of ele. CD influence: 0.01/(max_elevator)^2 for Zagi otherwise 0
   //@}

   /// @name Mass and inertia
   //@{
   SCALAR Mass;  // inertia
   SCALAR I_xx;  // inertia
   SCALAR I_yy;  // inertia
   SCALAR I_zz;  // inertia
   SCALAR I_xz;  // inertia
   //@}

   /// @name Gear and ground interaction
   //@{
   
   /**
    * Class containing all hard points/wheels
    */
   WheelSystem wheelsys;
   
   //@}
      
  private:
   
   void gear(TSimInputs* inputs);
   void aero_init();
   void aero(SCALAR dt, TSimInputs* inputs);

  private:
   
   /// @name written by constructor
   //@{
   
   /**
    * Propulsion system: batteries, shafts, engines, propellers.
    */
   Power::Power* power;
   
   /**
    * Velocity in trimmed flight; dead air.
    */
   float trimmedFlightVelocity;   
         
   //@}
   
   /// @name written by init, update
   //@{
   
   /**
    * velocity of airmass (steady winds)
    * north, east, down
    */
   CRRCMath::Vector3 v_V_local_airmass;
      
   /**
    * linear turbulence components, L frame 
    */
   CRRCMath::Vector3 v_V_gust_local;
   
   /**
    * Gradients of wind velocity, runway coordinates.
    * m_V_atmo_rwy.v[0][?] is U_atmo_?
    * m_V_atmo_rwy.v[1][?] is V_atmo_?
    * m_V_atmo_rwy.v[2][?] is W_atmo_?
    */
   CRRCMath::Matrix33 m_V_atmo_rwy;
   
   //@}
   
   /// @name written by aero
   //@{
   int stalling;
   
   /**
    * Force x/y/z
    */
   CRRCMath::Vector3 v_F_aero;
   
   /**
    * Moment
    * l/m/n <-> roll/pitch/yaw
    */
   CRRCMath::Vector3 v_M_aero;
   
   /**
    * velocity of body relative to airmass
    */
   CRRCMath::Vector3 v_V_body;
   
   //@}

   /// @name written by gear
   //@{
   
   CRRCMath::Vector3 v_F_gear;
   CRRCMath::Vector3 v_M_gear;
   
   //@}

   /// @name written by engine
   //@{
   CRRCMath::Vector3  v_F_engine;
   CRRCMath::Vector3  v_M_engine;
   //@}


};


#endif
