// -*- mode: c; mode: fold -*-
/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005 - 2009 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2006, 2008 - Jan Reucker
 *   Copyright (C) 2006 - Todd Templeton
 * 
 * This file is partially based on work by
 *   Jan Kansky
 *   Bruce Jackson
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
#ifndef FDM_LARCSIM_H
# define FDM_LARCSIM_H

# include <stdexcept>
# include <vector>
# include "../ls_types.h"
# include "../eom01/eom01.h"
# include "../../mod_math/vector3.h"
# include "../../mod_math/matrix33.h"
# include "../power/power.h"
# include "../gear01/gear.h"

/**
 * physical model for a fixed wing airplane
 * 
 * Jens Wilhelm Wulf, September 2005: attempt to (re)structure and simplify LaRCSim code.
 *
 * References originally given by Bruce Jackson:
 * 
 *                [ 1]    McFarland, Richard E.: "A Standard Kinematic Model
 *                        for Flight Simulation at NASA-Ames", NASA CR-2497,
 *                        January 1975
 *                [ 2]    ANSI/AIAA R-004-1992 "Recommended Practice: Atmos-
 *                        pheric and Space Flight Vehicle Coordinate Systems",
 *                        February 1992
 *                [ 3]    Beyer, William H., editor: "CRC Standard Mathematical
 *                        Tables, 28th edition", CRC Press, Boca Raton, FL, 1987,
 *                        ISBN 0-8493-0628-0
 *                [ 4]    Dowdy, M. C.; Jackson, E. B.; and Nichols, J. H.:
 *                        "Controls Analysis and Simulation Test Loop Environ-
 *                        ment (CASTLE) Programmer's Guide, Version 1.3",
 *                        NATC TM 89-11, 30 March 1989.
 *                [ 5]    Halliday, David; and Resnick, Robert: "Fundamentals
 *                        of Physics, Revised Printing", Wiley and Sons, 1974.
 *                        ISBN 0-471-34431-1
 *                [ 6]    Anon: "U. S. Standard Atmosphere, 1962"
 *                [ 7]    Anon: "Aeronautical Vest Pocket Handbook, 17th edition",
 *                        Pratt & Whitney Aircraft Group, Dec. 1977
 *                [ 8]    Stevens, Brian L.; and Lewis, Frank L.: "Aircraft
 *                        Control and Simulation", Wiley and Sons, 1992.
 *                        ISBN 0-471-61397-5
 * 
 * @author Bruce Jackson
 * @author Jens Wilhelm Wulf
 */
class CRRC_AirplaneSim_Larcsim : public EOM01
{
   friend class ModFDMInterface;
   friend class CRRC_AirplaneSim_DisplayMode;
   
  public:

   virtual bool   isStalling() { return(stalling != 0); };

   /**
    * Returns total CL.
    */
   virtual double getFlightCL() { return(flight_CL); };

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
   virtual double getTrimmedFlightVelocity() {return(trimmedFlightVelocity); };

   /**
    * Wingspan of the aircraft in feet
    */
   virtual double getWingspan() { return(B_ref); };
   
   /**
    * returns Z coordinate of lowest point
    */
   double getZLow() { return(wheels.getZLow()); };

   /**
    * This only tries to reload airplane parameters, but does not change states.
    */
   virtual int ReloadParams(SimpleXMLTransfer* xml, 
                            SimpleXMLTransfer* cfg);
  private:

   void LoadFromXML(SimpleXMLTransfer* xml, int nVerbosity);
  
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
   CRRC_AirplaneSim_Larcsim(const char* filename, FDMEnviroment* myEnv, SimpleXMLTransfer* cfg);

   /**
    * read from xml description
    */
   CRRC_AirplaneSim_Larcsim(SimpleXMLTransfer* xml, FDMEnviroment* myEnv, SimpleXMLTransfer* cfg);

   virtual ~CRRC_AirplaneSim_Larcsim();
   
  private:
      
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

   SCALAR flap_drag;
   SCALAR flap_lift;
   SCALAR flap_moment;
   SCALAR flap_eff_ratio;  // flap effectiveness at full deflection reduces by this ratio to account for non linearity at large 
                           // deflection angles. Suggested value: 1.0 for travel < 10°, down to 0.5-0.0 for 30°-60° travel
   SCALAR spoiler_drag;
   SCALAR spoiler_lift;
   SCALAR spoiler_moment;
   SCALAR retract_drag;
   SCALAR retract_lift;
   //@}

   /// @name Gear and ground interaction
   //@{

   /**
    * Vector containing all hard points/wheels
    */
   WheelSystem wheels;
         
   //@}

   /// @name Propeller and wing/fuselage/tail interaction
   //@{
 
   /**
    * Only this fraction of shaft torque is applied to the airframe
    */
   double effectivePropellerTorqueFactor;
   //@}
  
  private:

   void gear(TSimInputs* inputs, CRRCMath::Vector3& v_F, CRRCMath::Vector3& v_M);
   void aero(TSimInputs* inputs, 
             CRRCMath::Matrix33 m_V_atmo_rwy,
             CRRCMath::Vector3 v_R_omega_gust_body,
             CRRCMath::Vector3& v_F, CRRCMath::Vector3& v_M);
   void engine( SCALAR dt, TSimInputs* inputs, CRRCMath::Vector3& v_F, CRRCMath::Vector3& v_M);
   virtual void ls_step_init();
   
  private:   

   /// @name written by constructor
   //@{
   
   /**
    * Propulsion system: batteries, shafts, engines, propellers.
    */
   Power::Power* power;
   
   /**
    * Velocity in trimmed flight; dead air [ft/s].
    */
   float trimmedFlightVelocity;
   
   //@}
            
   /// @name written by aero
   //@{
   int stalling;   
         
   double flight_CL;   
   //@}
         
};

#endif
