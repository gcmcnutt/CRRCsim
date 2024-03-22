// -*- mode: c; mode: fold -*-
/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2006, 2008, 2009 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2006 - 2009 - Jan Reucker
 *   Copyright (C) 2006 - Todd Templeton
 *
 * This file is partially based on work by
 *   Jan Kansky
 *   Bruce Jackson
 * The respective methods have a header containing more details, see below.
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
#include "fdm_larcsim.h"

#include <math.h>
#include <iostream>

#include "../../mod_misc/ls_constants.h"
#include "../ls_geodesy.h"
#include "../../mod_misc/SimpleXMLTransfer.h"
#include "../../mod_misc/lib_conversions.h"
#include "../xmlmodelfile.h"

// 0, 1, 2
#define EOM_TEST 0

/**
 * *****************************************************************************
 */

void CRRC_AirplaneSim_Larcsim::initAirplaneState(double dRelVel,
                                                 double dPhi,
                                                 double dTheta,
                                                 double dPsi,
                                                 double X,
                                                 double Y,
                                                 double Z,
                                                 double R_X,
                                                 double R_Y,
                                                 double R_Z) 
{
  float flVelocity = dRelVel * getTrimmedFlightVelocity();

  Phi       = dPhi;         // bank/roll angle  [rad]
  Theta     = dTheta;       // pitch attitude angle [rad]
  Psi       = dPsi;         // heading angle [rad]

  Alpha     = 0;            // angle of attack            [rad]
  Beta      = 0;            // sideslip angle             [rad]

  {
    // see ls_aux(): 'determine location in runway coordinates'
    double slr, dummy;
    ls_geod_to_geoc( 0, 0, &slr, &dummy);

    Latitude  = X/slr;
    Longitude = Y/slr;
    Altitude  = -1*Z;
  }

  {
    // horizontal velocity
    float flVHor = flVelocity * cos(dTheta);
    // horizontal velocity has to be upwind:
    v_V_local.r[0]  = cos(Psi)*flVHor;      // local x-velocity (north)   [ft/s]
    v_V_local.r[1]  = sin(Psi)*flVHor;      // local y-velocity (east)    [ft/s]
    
    v_V_local_rel_ground.r[1] = v_V_local.r[1];
  }
  v_V_local.r[2]         = flVelocity * sin(-dTheta);          // local z-velocity (down)     [ft/s]
  
  v_R_omega_body    = CRRCMath::Vector3(R_X, R_Y, R_Z); // body rate   [rad/s]  
  v_V_dot_local     = CRRCMath::Vector3(); // local acceleration   [ft/s^2]
  
  CD_stall        = 0.05;   // drag coeff. during stalling    []

  ls_step_init();
  
  power->InitStates(v_V_wind_body * FT_TO_M);  
}


void CRRC_AirplaneSim_Larcsim::update(TSimInputs* inputs,
                                      double      dt,
                                      int         multiloop) 
{
  CRRCMath::Vector3 v_V_local_airmass;
  CRRCMath::Vector3 v_V_gust_body, v_R_omega_gust_body;
  CRRCMath::Matrix33 m_V_local_airmass_grad;  

  CRRCMath::Vector3 v_F_aero, v_F_engine, v_F_gear; // Force x/y/z
  CRRCMath::Vector3 v_M_aero, v_M_engine, v_M_gear; // l/m/n <-> roll/pitch/yaw

  int nAircraftOutsideWindfieldSim = 
    env->CalculateWind(v_P_CG_Rwy.r[0],        v_P_CG_Rwy.r[1],        v_P_CG_Rwy.r[2],
                       v_V_local_airmass.r[0], v_V_local_airmass.r[1], v_V_local_airmass.r[2]);
                       
  /**
   * Using a length of about roughly one half of the aircraft's
   * span to calculate wind gradients. 0.1 foot had been used before,
   * which leads to very high or zero gradients.
   */
  double delta_space = 0.5*getWingspan();
  
  nAircraftOutsideWindfieldSim |= 
    env->CalculateWindGrad(v_P_CG_Rwy.r[0], v_P_CG_Rwy.r[1], v_P_CG_Rwy.r[2], delta_space,
                           m_V_local_airmass_grad);

  if (nAircraftOutsideWindfieldSim)
  {
    env->AddLogMsg("Error: aircraft outside windfield simulation");
  }
    
  for (int n=0; n<multiloop; n++)
  {
    logNewline();
    
#if FDM_LOG_POS != 0
    logVal(v_P_CG_Rwy.r[0]);
    logVal(v_P_CG_Rwy.r[1]);
    logVal(v_P_CG_Rwy.r[2]);
    logVal(getPhi());    
    logVal(getTheta());
    logVal(getPsi());    
#endif
#if FDM_LOG_WIND_IN != 0
    logVal(v_V_local_airmass);
    logVal(m_V_atmo_rwy);
#endif
            
    ls_step(dt);

    // update wind turbulence linear & rotational velocities
    env->CalculateWindGust(dt, getAlt(), v_V_local_rel_airmass.length(), getWingspan(),
                           v_V_local_airmass, LocalToBody,
                           v_V_gust_body, v_R_omega_gust_body);
    
    ls_aux(v_V_local_airmass, v_V_gust_body);

    env->ControllerCallback(dt, this, inputs, &myInputs);
    
    aero(&myInputs, m_V_local_airmass_grad, v_R_omega_gust_body, v_F_aero, v_M_aero);
    
#if FDM_LOG_AERO_OUT != 0
    logVal(v_F_aero);
    logVal(v_M_aero);
#endif
    
    engine(dt, &myInputs, v_F_engine, v_M_engine);
    gear(&myInputs, v_F_gear, v_M_gear);

    // Only a fraction of the total rolling torque is not cancelled
    // by the aero effect of the prop wash on fuselage, wing and tail
    v_M_engine.r[0] *= effectivePropellerTorqueFactor;
    
    // Sum forces and moments at reference point (center of gravity)
    ls_accel(v_F_aero + v_F_engine + v_F_gear, v_M_aero + v_M_engine + v_M_gear);
  }
}


CRRC_AirplaneSim_Larcsim::CRRC_AirplaneSim_Larcsim(const char* filename, FDMEnviroment* myEnv, SimpleXMLTransfer* cfg) : EOM01("fdm_larcsim.dat", myEnv)
{
  // Previously there has been code to load an old-style .air-file. 
  // This has been removed as CRRCSim includes an automatic converter.
  SimpleXMLTransfer* fileinmemory = new SimpleXMLTransfer(filename);
  
  power = 0;
  LoadFromXML(fileinmemory, cfg->getInt("airplane.verbosity", 5));
  
  delete fileinmemory;
}


CRRC_AirplaneSim_Larcsim::CRRC_AirplaneSim_Larcsim(SimpleXMLTransfer* xml, FDMEnviroment* myEnv, SimpleXMLTransfer* cfg) : EOM01("fdm_larcsim.dat", myEnv)
{
  power = 0;
  LoadFromXML(xml, cfg->getInt("airplane.verbosity", 5));
}

void CRRC_AirplaneSim_Larcsim::LoadFromXML(SimpleXMLTransfer* xml, int nVerbosity)
{
  SimpleXMLTransfer* i;
  SimpleXMLTransfer* cfg = XMLModelFile::getConfig(xml);
  SimpleXMLTransfer* aero;
  
  // File format extension: an aero section inside of config takes
  // precedence over the general aero section.
  {
    int i = cfg->indexOfChild("aero");
    if (i >= 0)
      aero = cfg->getChildAt(i);
    else
      aero = xml->getChild("aero");
  }
  
  {
    double to_ft;
    
    switch (aero->getInt("units"))
    {
     case 0:
      to_ft = 1;
      break;
     case 1:
      to_ft = M_TO_FT;
      break;
     default:
      {
        throw std::runtime_error("Unknown units in aero");
      }
      break;
    }
    i = aero->getChild("ref");
    C_ref = i->getDouble("chord") * to_ft;
    B_ref = i->getDouble("span") * to_ft;
    S_ref = i->getDouble("area") * to_ft * to_ft;
    U_ref = i->getDouble("speed") * to_ft;
  }
    
  i = aero->getChild("misc");
  Alpha_0  = i->getDouble("Alpha_0");
  eta_loc  = i->getDouble("eta_loc");
  CG_arm   = i->getDouble("CG_arm");
  span_eff = i->getDouble("span_eff");
  
  i = aero->getChild("m");
  Cm_0  = i->getDouble("Cm_0");
  Cm_a  = i->getDouble("Cm_a");
  Cm_q  = i->getDouble("Cm_q");
  Cm_de = i->getDouble("Cm_de");
  
  i = aero->getChild("lift");
  CL_0    = i->getDouble("CL_0");
  CL_max  = i->getDouble("CL_max");
  CL_min  = i->getDouble("CL_min");
  CL_a    = i->getDouble("CL_a");
  CL_q    = i->getDouble("CL_q");
  CL_de   = i->getDouble("CL_de");
  CL_drop = i->getDouble("CL_drop");
  CL_CD0  = i->getDouble("CL_CD0");
  
  i = aero->getChild("drag");
  CD_prof  = i->getDouble("CD_prof");
  Uexp_CD  = i->getDouble("Uexp_CD");
  CD_stall = i->getDouble("CD_stall");
  CD_CLsq  = i->getDouble("CD_CLsq");
  CD_AIsq  = i->getDouble("CD_AIsq");
  CD_ELsq  = i->getDouble("CD_ELsq");
  
  i = aero->getChild("Y");
  CY_b  = i->getDouble("CY_b");
  CY_p  = i->getDouble("CY_p");
  CY_r  = i->getDouble("CY_r");
  CY_dr = i->getDouble("CY_dr");
  CY_da = i->getDouble("CY_da");
  
  i = aero->getChild("l");
  Cl_b  = i->getDouble("Cl_b");
  Cl_p  = i->getDouble("Cl_p");
  Cl_r  = i->getDouble("Cl_r");
  Cl_dr = i->getDouble("Cl_dr");
  Cl_da = i->getDouble("Cl_da");
  
  i = aero->getChild("n");
  Cn_b  = i->getDouble("Cn_b");
  Cn_p  = i->getDouble("Cn_p");
  Cn_r  = i->getDouble("Cn_r");
  Cn_dr = i->getDouble("Cn_dr");
  Cn_da = i->getDouble("Cn_da");
    
  flap_drag      = aero->getDouble("flap.drag", 0);
  flap_lift      = aero->getDouble("flap.lift", 0);
  flap_moment    = aero->getDouble("flap.moment", 0);
  flap_eff_ratio = aero->getDouble("flap.eff_ratio", 1.0);
  
  spoiler_drag   = aero->getDouble("spoiler.drag", 0);
  spoiler_lift   = aero->getDouble("spoiler.lift", 0);
  spoiler_moment = aero->getDouble("spoiler.moment", 0);
  
  retract_drag = aero->getDouble("retract.drag", 0);
  retract_lift = aero->getDouble("retract.lift", 0);
  
  effectivePropellerTorqueFactor = aero->getDouble("prop.torquefactor", 0.25);
  
  {
    double to_slug;
    double to_slug_ft_ft;
    
    i = cfg->getChild("mass_inertia");
    switch (i->getInt("units"))
    {
     case 0:    
      to_slug       = 1;
      to_slug_ft_ft = 1;
      break;
     case 1:
      to_slug       = KG_TO_SLUG;
      to_slug_ft_ft = KG_M_M_TO_SLUG_FT_FT;
      break;
     default:
      {
        throw std::runtime_error("Unknown units in mass_inertia");
      }
      break;
    }
    Mass  = i->getDouble("Mass") * to_slug;
    I_xx  = i->getDouble("I_xx") * to_slug_ft_ft;
    I_yy  = i->getDouble("I_yy") * to_slug_ft_ft;
    I_zz  = i->getDouble("I_zz") * to_slug_ft_ft;
    I_xz  = i->getDouble("I_xz") * to_slug_ft_ft;
  }
  
  wheels.init(xml, B_ref);

  // calculate velocity in trimmed flight,
  // approximately since Gravity & Density are yet unknown.
  // TrimmedFlightVelocity is used to initialise airplane state
  // and program crash if velocity is not reasonable
  // (e.g. trimmed CL <= 0)
{
    // crrc_aero says: Cm = Cm_0 + Cm_a*(Alpha-Alpha_0)
    // So Cm is zero at
    float alpha = ((Cm_a * Alpha_0) - Cm_0 ) / Cm_a;
    // crrc_aero says: CL  = CL_0 + CL_a*(Alpha - Alpha_0);
    float cl = CL_0 + CL_a * (alpha - Alpha_0);
    
    // sanity check, to avoid extreme (or negative) launch speed.
    // E.g. Cm_a data for biplane2.air is most likely wrong (should be < 0!)
    // so the equilibrium is achieved for a negative cl.
    if (cl < 0.1)
    {
      cl = 0.1;
      printf("airplane: Cm_a=%f   Alpha_0=%f  Cm_0=%f\n", Cm_a, Alpha_0, Cm_0);
      printf("airplane: CL_a=%f               CL_0=%f\n", CL_a,          CL_0);
      printf("airplane: alpha=%f  C_L=%f\n", alpha, cl);
    }
    
    // Gravity & Density yet not initialised: assume 0 height
    float Gravity = env->GetG(0.0);
    float Density = env->GetRho(0.0);
    
    // v = sqrt(m * g * 2 / (S * CL * rho))
    trimmedFlightVelocity = sqrt(Mass * Gravity * 2.0 / (S_ref * cl * Density));
  }
  
  // possibly there is no power description:
  if (power == 0)
  {
    if (cfg->indexOfChild("power") < 0)
    {
      power = new Power::Power();
    }
    else
    {
      power = new Power::Power(cfg, nVerbosity);
    }
  }
  else if (cfg->indexOfChild("power") >= 0)
    power->ReloadParams(cfg, nVerbosity);    

  if (nVerbosity > 1)
  {
    std::cout << "--- Airplane description: ---------------------------------------\n";
    std::cout << "  Wingspan:                   " << (B_ref*FT_TO_M) << " m\n";
    std::cout << "  Mass:                       " << (Mass*SLUG_TO_KG) << " kg\n";
    std::cout << "  Velocity in trimmed flight: " << (trimmedFlightVelocity*FT_TO_M) << " m/s\n";
    std::cout << "-----------------------------------------------------------------\n";
  }
}

CRRC_AirplaneSim_Larcsim::~CRRC_AirplaneSim_Larcsim()
{
  delete power;
}

/** \brief Calculate influence of gear and hardpoints
 *
 *  This method calculates the forces and moments on
 *  the airplane's body caused by the gear or other
 *  hardpoints touching the ground.
 *
 *  \param inputs   Current control inputs
 */
void CRRC_AirplaneSim_Larcsim::gear(TSimInputs* inputs, CRRCMath::Vector3& v_F, CRRCMath::Vector3& v_M)
{
  wheels.update(inputs,
                env,
                LocalToBody,
                v_P_CG_Rwy,
                v_R_omega_body,
                v_V_local_rel_ground,
                Psi);

  v_F = wheels.getForces();
  v_M = wheels.getMoments();
}


void CRRC_AirplaneSim_Larcsim::engine( SCALAR dt, TSimInputs* inputs, CRRCMath::Vector3& v_F, CRRCMath::Vector3& v_M)
{
  v_F = CRRCMath::Vector3();
  v_M = CRRCMath::Vector3();
  
  inputs->pitch = 1;
  power->step(dt, inputs, v_V_wind_body*FT_TO_M, &v_F, &v_M);

  // Convert SI to that other buggy system.
  v_F *= N_TO_LBF;
  v_M *= NM_TO_LBFFT;
}


/**
 * This method is based on crrc_aero.c in the initial version of CRRCSim. 
 * It did not contain a copyright or author note and was committed to CVS 
 * by Jan Kansky.
 * 
 * Calculate forces and moments for the current time step.
 */
void CRRC_AirplaneSim_Larcsim::aero(TSimInputs* inputs, 
                                    CRRCMath::Matrix33 m_V_atmo_rwy,
                                    CRRCMath::Vector3 v_R_omega_gust_body,
                                    CRRCMath::Vector3& v_F, CRRCMath::Vector3& v_M)
{
  SCALAR elevator, aileron, rudder, flap, spoiler, gear_ext;
  SCALAR flap_eff_factor;
  SCALAR Phat, Qhat, Rhat; // dimensionless rotation rates
  SCALAR CL_left, CL_cent, CL_right; // CL values across the span
  SCALAR dCL_left,dCL_cent,dCL_right; // stall-induced CL changes
  SCALAR dCD_left,dCD_cent,dCD_right; // stall-induced CD changes
  SCALAR dCl,dCn,dCl_stall,dCn_stall; // stall-induced roll,yaw moments
  SCALAR dCm_stall; // Stall induced pitching moment.
  SCALAR CL_flap_offset, CL_offset, CL_wing, CD_all, CD_scaled, Cl_w;
  SCALAR Cl_r_mod,Cn_p_mod;
  SCALAR CL,CD,Cl,Cm,Cn;
  SCALAR QS;
  
  CRRCMath::Vector3 C_xyz;

  CRRCMath::Vector3 v_R_omega_atmo;
  
  CRRCMath::Matrix33 m_V_body;

  SCALAR Cos_alpha, Sin_alpha, Cos_beta;

  Cos_alpha = cos(Alpha);
  Sin_alpha = sin(Alpha);
  Cos_beta  = cos(Beta);

  elevator = inputs->elevator;
  aileron  = inputs->aileron;
  rudder   = inputs->rudder;
  flap     = inputs->flap;
  spoiler  = inputs->spoiler;
  
  /* To simulate the non linearity of flap effectiveness for large flap
   * deflections (e.g. landing flap) a flap effectiveness ratio can be
   * defined, giving the ratio R of flap derivatives at full deflection
   * (i.e. flap = 0.5) vs at zero deflection. 
   * R=1 for standard constant effectiveness (linear behaviour).
   * The resulting flap effectiveness factor for the current deflection
   * is then computed as k = 1 - (1 - R)*flap
   */
  flap_eff_factor = 1. - (1. - flap_eff_ratio)*flap; 
   
  /* The retract input is 0.0 for fully extended gear, 1.0 for fully
   * retracted gear.
   */
  gear_ext = 1.0 - inputs->retract;

  /* transform wind velocity gradient from local to body frame */
  m_V_body = LocalToBody * (m_V_atmo_rwy * LocalToBody.trans());
  
  /* set rotation rates of airmass motion includin wind turbulence effect */
  v_R_omega_atmo.r[0] =  m_V_body.v[2][1] + v_R_omega_gust_body.r[0];
  v_R_omega_atmo.r[1] = -m_V_body.v[2][0] + v_R_omega_gust_body.r[1];
  v_R_omega_atmo.r[2] =  m_V_body.v[1][0] + v_R_omega_gust_body.r[2];

  if (V_rel_wind != 0)
  {
    /* set net effective dimensionless rotation rates */
    // jww: the comment above is misleading. The unit of those values must be rad!
    Phat = (v_R_omega_body.r[0] - v_R_omega_atmo.r[0]) * B_ref / (2.0*V_rel_wind);
    Qhat = (v_R_omega_body.r[1] - v_R_omega_atmo.r[1]) * C_ref / (2.0*V_rel_wind);
    Rhat = (v_R_omega_body.r[2] - v_R_omega_atmo.r[2]) * B_ref / (2.0*V_rel_wind);
  }
  else
  {
    Phat=0;
    Qhat=0;
    Rhat=0;
  }

  /* compute local CL at three spanwise locations */
  CL_flap_offset = flap_lift*flap_eff_factor*flap;
  CL_offset = CL_flap_offset + spoiler_lift*spoiler + retract_lift*gear_ext;
  CL_left   = CL_0 + CL_a*(Alpha - Alpha_0 - Phat*eta_loc) + CL_offset;
  CL_cent   = CL_0 + CL_a*(Alpha - Alpha_0               ) + CL_offset;
  CL_right  = CL_0 + CL_a*(Alpha - Alpha_0 + Phat*eta_loc) + CL_offset;

  /* set CL-limit changes */
  dCL_left  = 0.;
  dCL_cent  = 0.;
  dCL_right = 0.;

  // careful treatment of both positive and negative stall.
  // The CL_max & CL_min values specified in input must be those of a clean
  // wing, i.e. for flap=spoiler=gear_ext=0.
  // The effect of flap is to add an offset to the lift, (i.e. change the 
  // zero-lift angle of attack) but the CL_max & CL_min are not offset by 
  // a similar amount: a crude approximation is that they are only offset
  // by half of the amount.  
  stalling=0;
  {
    SCALAR CL_max_flap = CL_max + 0.5*CL_flap_offset;
    SCALAR CL_min_flap = CL_min + 0.5*CL_flap_offset;
    
    if (CL_left  > CL_max_flap)
    {
      dCL_left  = CL_max_flap - CL_left - CL_drop;
      stalling=1;
    }
    if (CL_cent  > CL_max_flap)
    {
      dCL_cent  = CL_max_flap - CL_cent - CL_drop;
      stalling=1;
    }
    if (CL_right > CL_max_flap)
    {
      dCL_right = CL_max_flap - CL_right - CL_drop;
      stalling=1;
    }
    if (CL_left  < CL_min_flap)
    {
      dCL_left  = CL_min_flap - CL_left - CL_drop*CL_min/CL_max;
      stalling=1;
    }
    if (CL_cent  < CL_min_flap)
    {
      dCL_cent  = CL_min_flap - CL_cent - CL_drop*CL_min/CL_max;
      stalling=1;
    }
    if (CL_right < CL_min_flap)
    {
      dCL_right  = CL_min_flap - CL_right - CL_drop*CL_min/CL_max;
      stalling=1;
    }
  }
  
  /* set average wing CL */
  CL_wing = 0.5*(CL_cent + dCL_cent)
    + 0.25*(CL_left + dCL_left) + 0.25*(CL_right + dCL_right);

  /* total CL, with pitch rate and control contributions */
  CL = (CL_wing + CL_q*Qhat 
    + CL_de*elevator
    )*Cos_alpha;
  
  /* save flight CL for public access */
  flight_CL = CL;
  
  /* correct profile CD for CL dependence and controls dependence */
  // Note quadratic effect of flap deflection
  CD_all = CD_prof
    + CD_CLsq*(CL_wing-CL_CD0)*(CL_wing-CL_CD0)
    + CD_AIsq*aileron*aileron
    + CD_ELsq*elevator*elevator
    + flap_drag*flap_eff_factor*flap*flap
    + spoiler_drag*spoiler
    + retract_drag*gear_ext;
  
  /* scale profile CD with Reynolds number via simple power law */
  if (V_rel_wind > 0.1)
  {
    CD_scaled = CD_all*pow(((double)V_rel_wind/(double)U_ref),Uexp_CD);
  }
  else
  {
    CD_scaled = CD_all;
  }

  /* Scale lateral cross-coupling derivatives with wing CL */
  Cl_r_mod = Cl_r*CL_wing/CL_0;
  Cn_p_mod = Cn_p*CL_wing/CL_0;

  /* stall-caused CD */
  // CD_stall is max delta CD due to a (deep) stall, assumed to be one
  // which causes a drop of 0.4 (e.g.) from linear CL curve.
  // The harder the stall (that is CL_drop) the quicker this drag level
  // is approached.
  dCD_left  = CD_stall*(1.0 - exp(-4.0*fabs(dCL_left )/0.4));
  dCD_cent  = CD_stall*(1.0 - exp(-4.0*fabs(dCL_cent )/0.4));
  dCD_right = CD_stall*(1.0 - exp(-4.0*fabs(dCL_right)/0.4));

  /* asymetric stall will cause roll and yaw moments */
  dCl = -0.25*(dCL_right - dCL_left)*eta_loc;
  dCn =  0.25*(dCD_right - dCD_left)*eta_loc;

  /* stall-caused moments in body axes */
  dCl_stall = dCl*Cos_alpha - dCn*Sin_alpha;
  dCn_stall = dCl*Sin_alpha + dCn*Cos_alpha;

  /* stall-caused pitching moment in body axes */
  dCm_stall = (0.25*dCL_left + 0.5*dCL_cent + 0.25*dCL_right)*CG_arm;

  /* roll moment due to wing only */
  Cl_w = Cl_b*Beta  + Cl_p*Phat + Cl_r_mod*Rhat
    + dCl_stall  + Cl_da*aileron;

  /* total CD, with induced and stall contributions */
  // Roll moment is due to asymmetric lift contributions
  // adding to induced drag. Approximately |Cl_w| = 0.5*|dCL|
  CD = CD_scaled
    + pow(fabs(CL) + fabs(2.0*Cl_w),2)*S_ref/(B_ref*B_ref*M_PI*span_eff)
      + 0.25*dCD_left + 0.5*dCD_cent + 0.25*dCD_right;

  /* total forces in body axes */
  C_xyz.r[0] = -CD*Cos_alpha + CL*Sin_alpha*Cos_beta*Cos_beta;
  C_xyz.r[2] = -CD*Sin_alpha - CL*Cos_alpha*Cos_beta*Cos_beta;
  C_xyz.r[1] = CY_b*Beta  + CY_p*Phat + CY_r*Rhat + CY_dr*rudder;

  /* total moments in body axes */
  // Flap contribution to Cm zeroed if stalling, since major
  // effect is coming from lift variation rather than airfoil cm
  Cl = Cl_b*Beta + Cl_p*Phat + Cl_r_mod*Rhat
    + Cl_dr*rudder + Cl_da*aileron
    + dCl_stall;
  Cn = Cn_b*Beta + Cn_p_mod*Phat + Cn_r*Rhat
    + Cn_dr*rudder + Cn_da*aileron
    + dCn_stall;
  Cm = Cm_0 + Cm_a*(Alpha-Alpha_0) + Cm_q*Qhat 
    + Cm_de*elevator
    + dCm_stall
    + flap_moment*flap_eff_factor*flap*(1.0 - stalling)
    + spoiler_moment*spoiler;

  /* set dimensional forces and moments */
  QS = 0.5*Density*V_rel_wind*V_rel_wind * S_ref;

  v_F = C_xyz * QS;

  v_M.r[0] = Cl * QS * B_ref;
  v_M.r[1] = Cm * QS * C_ref;
  v_M.r[2] = Cn * QS * B_ref;
  
#if (EOM_TEST == 1)
  {
    double ele = 0.8*(-inputs->elevator);
    double ail = 0.8*(inputs->aileron);
    double rud = 0.8*(0.5*inputs->aileron);
   
    v_F = CRRCMath::Vector3();
    v_M = CRRCMath::Vector3(ail, ele, rud);
    stalling = 0;
  }
#endif  
  
}


void CRRC_AirplaneSim_Larcsim::ls_step_init()
{
  CRRCMath::Vector3 v_F_aero, v_F_engine, v_F_gear; // Force x/y/z
  CRRCMath::Vector3 v_M_aero, v_M_engine, v_M_gear; // l/m/n <-> roll/pitch/yaw
  
  TSimInputs ZeroInput = TSimInputs();
  
  EOM01::ls_step_init();
  
  /*    Initialize vehicle model                        */
  ls_aux(CRRCMath::Vector3(), CRRCMath::Vector3());

  aero(&ZeroInput, CRRCMath::Matrix33(), CRRCMath::Vector3(), v_F_aero, v_M_aero);
  engine(0, &ZeroInput, v_F_engine, v_M_engine);
  gear(&ZeroInput, v_F_gear, v_M_gear);

  /*    Calculate initial accelerations */
  ls_accel(v_F_aero + v_F_engine + v_F_gear, v_M_aero + v_M_engine + v_M_gear);

  /* Initialize auxiliary variables */
  ls_aux(CRRCMath::Vector3(), CRRCMath::Vector3());
}


double CRRC_AirplaneSim_Larcsim::getPropFreq() 
{
  return(power->getPropFreq());
}


int CRRC_AirplaneSim_Larcsim::ReloadParams(SimpleXMLTransfer* xml,
                                           SimpleXMLTransfer* cfg)
{
  LoadFromXML(xml, cfg->getInt("airplane.verbosity", 5));
  
  return(1);
}
