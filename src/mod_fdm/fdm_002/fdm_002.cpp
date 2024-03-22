// -*- mode: c; mode: fold -*-
/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2006, 2008 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2006, 2007 - Jan Reucker
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
#include "fdm_002.h"

#include <math.h>
#include <iostream>

#include "../../mod_misc/ls_constants.h"
#include "../../mod_misc/lib_conversions.h"
#include "../xmlmodelfile.h"


// 0
// 1
// 2
// 3: simple test of drawing transformation
#define EOM_TEST 0

#if (EOM_TEST == 2)
int nLogCnt;
int nStep;
#endif

void CRRC_AirplaneSim_002::initAirplaneState(double dRelVel,
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
  float       flVelocity = dRelVel * getTrimmedFlightVelocity();
  
  // About inertia:
  //    |  I_xx  -I_xy  -I_xz |
  //    | -I_xy   I_yy  -I_yz |
  //    | -I_xz  -I_yz   I_zz |
  // I_xx, I_yy, I_zz are needed in any case.
  // If the plane is symmetric about its y-axis, I_yz=I_xy=0.
  //    |  I_xx   0     -I_xz |
  //    |  0      I_yy   0    |
  //    | -I_xz   0      I_zz |
  // 
  // What about the sign of I_xz?
  //    I_xz = integral x * z dm
  // 
  // Four possibilities:
  //   1. positive I_xz  
  //      1.1 in front of cg (x>0) and below cg (z>0)
  //      1.2 behind cg (x<0) and over cg (z<0)
  //   2. negative I_xz
  //      2.1 behind cg (x<0) and below cg (z>0)
  //      2.2 in front of cg (x>0) and over cg (z<0)
  //     
  //                    |
  //                    |
  //        negative    |   positive
  //                    |
  //                    |
  //   x <--------------+-----------------
  //                    |
  //                    |
  //        positive    |   negative
  //                    |
  //                    |
  //                    v z
  // 
  // It's easy to see that the vertical fin of the rudder gives a positive I_xz.
  // Usually the nose of the fuselage is heavy/long and below the cg, which leads to 
  // a positive I_xz, too.
  // But one is free to choose the direction of the body axes (let the x-axis point a little 
  // higher or lower from the cg) and thereby change I_xz.
  
  // jwtodo: Anstellwinkel gescheit einbauen!
  eom = EOM_6DOF(CRRCMath::Vector3(X, Y, Z),
                 CRRCMath::Vector3(dPhi, dTheta, dPsi),
                 CRRCMath::Vector3(flVelocity, 0, 0),
                 Mass,
                 CRRCMath::Matrix33( I_xx,    0,    -I_xz,
                                     0,       I_yy,  0,
                                    -I_xz,    0,     I_zz));

#if (EOM_TEST != 0)
  eom.setGravity(0);
#else
  eom.setGravity(env->GetG(-eom.pos.val.r[2]));
#endif
  
  v_V_local_airmass = CRRCMath::Vector3(); // local velocity of steady airmass   [ft/s]
  
  // jwtodo: seems this can be used to simulate wind gusts
  v_V_gust_local  = CRRCMath::Vector3();
  
  CD_stall        = 0.05;   // drag coeff. during stalling    []
  
  m_V_atmo_rwy = CRRCMath::Matrix33();
  
  aero_init();
  
  power->InitStates(CRRCMath::Vector3()); // todo
}


void CRRC_AirplaneSim_002::update(TSimInputs* inputs,
                                  double      dt,
                                  int         multiloop) 
{
  double V_north_xp,V_east_xp,V_down_xp;   // temp vars for calculating wind grad.
  double V_north_yp,V_east_yp,V_down_yp;
  double V_north_zp,V_east_zp,V_down_zp;
  double V_north_xm,V_east_xm,V_down_xm;
  double V_north_ym,V_east_ym,V_down_ym;
  double V_north_zm,V_east_zm,V_down_zm;

  /**
   * Using a length of about roughly one half of the aircraft's
   * span to calculate wind gradients. 0.1 foot had been used before,
   * which leads to very high or zero gradients.
   */
  double delta_space = 0.5*getWingspan();
  
  // Anpassung an LaRCSim: dort ist H=-Z
  CRRCMath::Vector3 v_P_CG_Rwy = eom.pos.val;
  
  int   nAircraftOutsideWindfieldSim =
    env->CalculateWind(v_P_CG_Rwy.r[0]+delta_space, v_P_CG_Rwy.r[1],             v_P_CG_Rwy.r[2],
                       V_north_xp,                  V_east_xp,                   V_down_xp) |
    env->CalculateWind(v_P_CG_Rwy.r[0],             v_P_CG_Rwy.r[1]+delta_space, v_P_CG_Rwy.r[2],
                       V_north_yp,                  V_east_yp,                   V_down_yp) |
    env->CalculateWind(v_P_CG_Rwy.r[0],             v_P_CG_Rwy.r[1],             v_P_CG_Rwy.r[2]+delta_space,
                       V_north_zp,                  V_east_zp,                   V_down_zp) |
    env->CalculateWind(v_P_CG_Rwy.r[0]-delta_space, v_P_CG_Rwy.r[1],             v_P_CG_Rwy.r[2],
                       V_north_xm,                  V_east_xm,                   V_down_xm) |
    env->CalculateWind(v_P_CG_Rwy.r[0],             v_P_CG_Rwy.r[1]-delta_space, v_P_CG_Rwy.r[2],
                       V_north_ym,                  V_east_ym,                   V_down_ym) |
    env->CalculateWind(v_P_CG_Rwy.r[0],             v_P_CG_Rwy.r[1],             v_P_CG_Rwy.r[2]-delta_space,
                       V_north_zm,                  V_east_zm,                   V_down_zm) |
    env->CalculateWind(v_P_CG_Rwy.r[0],             v_P_CG_Rwy.r[1],             v_P_CG_Rwy.r[2],
                       v_V_local_airmass.r[0],      v_V_local_airmass.r[1],      v_V_local_airmass.r[2]);
  
  if (nAircraftOutsideWindfieldSim)
  {
    // todo: some error message?
  }
  
  // Gradients are calculated from symmetric pairs to get symmetric behaviour.
  m_V_atmo_rwy.v[0][0] = (V_north_xp - V_north_xm)/(2*delta_space);
  m_V_atmo_rwy.v[0][1] = (V_north_yp - V_north_ym)/(2*delta_space);
  m_V_atmo_rwy.v[0][2] = (V_north_zp - V_north_zm)/(2*delta_space);
  m_V_atmo_rwy.v[1][0] = (V_east_xp  - V_east_xm) /(2*delta_space);
  m_V_atmo_rwy.v[1][1] = (V_east_yp  - V_east_ym) /(2*delta_space);
  m_V_atmo_rwy.v[1][2] = (V_east_zp  - V_east_zm) /(2*delta_space);
  m_V_atmo_rwy.v[2][0] = (V_down_xp  - V_down_xm) /(2*delta_space);
  m_V_atmo_rwy.v[2][1] = (V_down_yp  - V_down_ym) /(2*delta_space);
  m_V_atmo_rwy.v[2][2] = (V_down_zp  - V_down_zm) /(2*delta_space);

#if (EOM_TEST == 2)
  switch (nStep)
  {
   case 0:
    if (inputs->elevator < 0)
      nStep = 1;
    multiloop = 0;
    break;
   case 2:
    if (inputs->elevator > 0)
      nStep = 3;
    multiloop = 0;
    break;
   case 4:
    nStep     = 0;
    multiloop = 0;
   default:
    break;
  }
#endif
  
  for (int n=0; n<multiloop; n++)
  {        
    logNewline();
    
#if FDM_LOG_POS != 0
    logVal(eom.pos.val);
    logVal(getPhi());    
    logVal(getTheta());
    logVal(getPsi());    
#endif    
#if FDM_LOG_WIND_IN != 0
    logVal(v_V_local_airmass);
    logVal(m_V_atmo_rwy);
#endif    
    
    env->ControllerCallback(dt, this, inputs, &myInputs);
    
#if (EOM_TEST != 2)
    aero( dt, &myInputs);
#endif

#if FDM_LOG_AERO_OUT != 0
    logVal(v_F_aero);
    logVal(v_M_aero);
#endif
    
    {
      v_F_engine = CRRCMath::Vector3();
      v_M_engine = CRRCMath::Vector3();

#if (EOM_TEST != 2)
      power->step(dt, &myInputs, v_V_body*FT_TO_M, &v_F_engine, &v_M_engine);
#endif
      
      // Convert SI to that other buggy system.
      v_F_engine *= N_TO_LBF;
      v_M_engine *= NM_TO_LBFFT;
    }
    gear(&myInputs);

#if (EOM_TEST == 2)
    CRRCMath::Vector3 v_F, v_M_cg;
    
    switch ((nLogCnt>>5) % 6)
    {
     case 0:
      v_F    = CRRCMath::Vector3(1, 0, 0);
      v_M_cg = CRRCMath::Vector3(0, 0, 0);
      break;
     case 1:
      v_F    = CRRCMath::Vector3(0, 0, 0);
      v_M_cg = CRRCMath::Vector3(0, 2, 0);
      break;
     case 2:
      v_F    = CRRCMath::Vector3(0, 0, 3);
      v_M_cg = CRRCMath::Vector3(0, 0, 0);
      break;
     case 3:
      v_F    = CRRCMath::Vector3(0, 0, 0);
      v_M_cg = CRRCMath::Vector3(1, 0, 0);
      break;
     case 4:
      v_F    = CRRCMath::Vector3(0, 2, 0);
      v_M_cg = CRRCMath::Vector3(0, 0, 0);
      break;
     case 5:
      v_F    = CRRCMath::Vector3(0, 0, 0);
      v_M_cg = CRRCMath::Vector3(0, 0, 3);
      break;
    }
  
    {
      double d = sin(M_PI*(nLogCnt&0x1F)/32.0);
      
      v_F    *= d*d;
      v_M_cg *= d*d;
    }
    
    if ((nLogCnt&0x1F) == 0x1F)
    {
      nStep++;
      std::cout << "\n";
    }
    
    if (  ((nLogCnt>>5)/6) % 2)
    {
      v_F    *= -1;
      v_M_cg *= -1;
    }
    
//    v_F *= 0;

    v_F.print("F=", ", ");
    v_M_cg.print("M=", "\n");
    
    eom.step(dt, v_F, v_M_cg);
    eom.conv.updateEuler();
#else
    eom.step(dt, 
             v_F_aero + v_F_engine + v_F_gear,
             v_M_aero + v_M_engine + v_M_gear);
    
    // Update the position they are interested in outside of this fdm. I have to
    // do it inside of the loop because gear() also needs it.
    eom.conv.updateEuler();
#endif
    
//    eom.conv.convTest1();
  }
     
  /*
  static int cnt  = 0;
  static int posi = 0;
  bool fChange = false;
   */
  
  /*
  posi = 0;
  if (inputs->aileron > 0)
    posi |= 1;
  if (inputs->elevator > 0)
    posi |= 2;
  fChange = true;
   */
  
  /*
  cnt += multiloop;
  if (cnt > 6000)
  {
    cnt -= 6000;
    posi++;
    posi %= 4;
    fChange = true;
  }
   */
  
  /*
  switch (posi)
  {
   case 0:
    pos->d_x = 20;
    pos->d_y = 20;
    pos->d_z = -5;
    break;
   case 1:
    pos->d_x = -20;
    pos->d_y = 20;
    pos->d_z = -6;
    break;
   case 3:
    pos->d_x = -20;
    pos->d_y = -20;
    pos->d_z = -3;
    break;
   default:
    pos->d_x = 20;
    pos->d_y = -20;
    pos->d_z = -4;
    break;
  }
   */

  /*
  pos->d_x = inputs->elevator*30;
  pos->d_y = inputs->aileron*30;
  pos->d_z = -0.5*sqrt(pos->d_x*pos->d_x + pos->d_y*pos->d_y);
  
  eom.conv.euler.r[0] = (M_PI/2) -1.0 *atan2(pos->d_x, pos->d_y);
  eom.conv.euler.r[1] = (3.0*M_PI/4.0); // (M_PI/2) -1.0 *atan2(pos->d_x, pos->d_y);
  eom.conv.euler.r[2] = (M_PI/2); // (M_PI/2) -1.0 *atan2(pos->d_x, pos->d_y);
    */
  
  // fChange = true;
  
  /*
  if (fChange)
  {
    std::cout << pos->d_x << " " << pos->d_y << " " << pos->d_z << " "
      << eom.conv.euler.r[0] << " " << eom.conv.euler.r[1] << " " << eom.conv.euler.r[2] << "\n";
  }
  */
  
  
#if (EOM_TEST == 3)
  eom.pos.val      = CRRCMath::Vector3(10, 20, -5);
  eom.conv.euler   = CRRCMath::Vector3(0.25*M_PI, 0.25*M_PI, 0.25*M_PI);
#endif
  
}


CRRC_AirplaneSim_002::CRRC_AirplaneSim_002(const char* filename, FDMEnviroment* myEnv) : FDMBase("fdm_002.dat", myEnv)
{
  // Previously there has been code to load an old-style .air-file. 
  // This has been removed as CRRCSim includes an automatic converter.
  SimpleXMLTransfer* fileinmemory = new SimpleXMLTransfer(filename);
  
  LoadFromXML(fileinmemory);
  
  delete fileinmemory;
}

CRRC_AirplaneSim_002::CRRC_AirplaneSim_002(SimpleXMLTransfer* xml, FDMEnviroment* myEnv) : FDMBase("fdm_larcsim.dat", myEnv)
{
  LoadFromXML(xml);
}

void CRRC_AirplaneSim_002::LoadFromXML(SimpleXMLTransfer* xml)
{
  SimpleXMLTransfer* i;
  SimpleXMLTransfer* cfg = XMLModelFile::getConfig(xml);
  
  {
    double to_ft;
    
    switch (xml->getInt("aero.units"))
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
    i = xml->getChild("aero.ref");
    C_ref = i->getDouble("chord") * to_ft;
    B_ref = i->getDouble("span") * to_ft;
    S_ref = i->getDouble("area") * to_ft * to_ft;
    U_ref = i->getDouble("speed") * to_ft;
  }
  
  i = xml->getChild("aero.misc");
  Alpha_0  = i->getDouble("Alpha_0");
  eta_loc  = i->getDouble("eta_loc");
  CG_arm   = i->getDouble("CG_arm");
  span_eff = i->getDouble("span_eff");
  
  i = xml->getChild("aero.m");
  Cm_0  = i->getDouble("Cm_0");
  Cm_a  = i->getDouble("Cm_a");
  Cm_q  = i->getDouble("Cm_q");
  Cm_de = i->getDouble("Cm_de");
  
  i = xml->getChild("aero.lift");
  CL_0    = i->getDouble("CL_0");
  CL_max  = i->getDouble("CL_max");
  CL_min  = i->getDouble("CL_min");
  CL_a    = i->getDouble("CL_a");
  CL_q    = i->getDouble("CL_q");
  CL_de   = i->getDouble("CL_de");
  CL_drop = i->getDouble("CL_drop");
  CL_CD0  = i->getDouble("CL_CD0");
  
  i = xml->getChild("aero.drag");
  CD_prof  = i->getDouble("CD_prof");
  Uexp_CD  = i->getDouble("Uexp_CD");
  CD_stall = i->getDouble("CD_stall");
  CD_CLsq  = i->getDouble("CD_CLsq");
  CD_AIsq  = i->getDouble("CD_AIsq");
  CD_ELsq  = i->getDouble("CD_ELsq");
  
  i = xml->getChild("aero.Y");
  CY_b  = i->getDouble("CY_b");
  CY_p  = i->getDouble("CY_p");
  CY_r  = i->getDouble("CY_r");
  CY_dr = i->getDouble("CY_dr");
  CY_da = i->getDouble("CY_da");
  
  i = xml->getChild("aero.l");
  Cl_b  = i->getDouble("Cl_b");
  Cl_p  = i->getDouble("Cl_p");
  Cl_r  = i->getDouble("Cl_r");
  Cl_dr = i->getDouble("Cl_dr");
  Cl_da = i->getDouble("Cl_da");
  
  i = xml->getChild("aero.n");
  Cn_b  = i->getDouble("Cn_b");
  Cn_p  = i->getDouble("Cn_p");
  Cn_r  = i->getDouble("Cn_r");
  Cn_dr = i->getDouble("Cn_dr");
  Cn_da = i->getDouble("Cn_da");
  
  
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
  
  wheelsys.init(xml, B_ref);
  
  // calculate velocity in trimmed flight
  {
    // crrc_aero says: Cm = Cm_0 + Cm_a*(Alpha-Alpha_0)
    // So Cm is zero at
    float alpha = ((Cm_a * Alpha_0) - Cm_0 ) / Cm_a;
    // crrc_aero says: CL  = CL_0 + CL_a*(Alpha - Alpha_0);
    float cl = CL_0 + CL_a * (alpha - Alpha_0);
    
    // sanity check: currently this method does not really work for biplane2.air, as
    // a negative cl is computed. ???
    if (cl < 0.2)
    {
      cl = 0.2;
      printf("airplane: Cm_a=%f   Alpha_0=%f  Cm_0=%f\n", Cm_a, Alpha_0, Cm_0);
      printf("airplane: CL_a=%f               CL_0=%f\n", CL_a,          CL_0);
      printf("airplane: alpha=%f  C_L=%f\n", alpha, cl);
    }
    
    // v = sqrt(m * g * 2 / (F * CL * rho))
    // I'll use conversions to SI units, just to understand the numbers...
    const float foot = 0.3048;               // m
    const float slug = 14.5939041995;        // kg
    float m    = Mass * slug;                // kg
    float g    = 9.81;                       // m/s^2
    float F    = S_ref * (foot*foot);        // m^2
    float rho  = 1.225;                      // kg/m^3
    float v = sqrt(m * g * 2 / (F * cl * rho));
    std::cout << "trimmedFlightVelocity = " << v << " m/s\n";
    trimmedFlightVelocity = v / foot; // ft/s
  }
  
  // possibly there is no power description:
  if (cfg->indexOfChild("power") < 0)
  {
    power = new Power::Power();
  }
  else
  {
    power = new Power::Power(cfg);
  }  
}

CRRC_AirplaneSim_002::~CRRC_AirplaneSim_002()
{
  delete power;
}

double CRRC_AirplaneSim_002::getPhi()
{
  return(eom.conv.euler.r[0]);
}

double CRRC_AirplaneSim_002::getTheta()
{
  return(eom.conv.euler.r[1]);
}

double CRRC_AirplaneSim_002::getPsi()
{
  return(eom.conv.euler.r[2]);
}

CRRCMath::Vector3 CRRC_AirplaneSim_002::getPos()
{
  return(eom.pos.val);
}


/**
 * 
 * jwtodo
 *  gut:
 *   skimmer
 *   arkjan
 *   apogee
 *   allegro
 *   mav
 *   delta*
 *   zip60
 *   sport
 *   viele andere
 * 
 *  schlecht:
 *   superzagi
 *   quickie
 *   zagi
 *   zagi-xs
 * 
 */
/** \brief Calculate influence of gear and hardpoints
 *
 *  This method calculates the forces and moments on
 *  the airplane's body caused by the gear or other
 *  hardpoints touching the ground.
 *
 *  \param inputs   Current control inputs
 */
void CRRC_AirplaneSim_002::gear(TSimInputs* inputs)      
{
  wheelsys.update(inputs,
                  env,
                  eom.conv.mat,
                  eom.pos.val,
                  eom.angvel.val,
                  eom.conv.local(eom.vel.val),
                  eom.conv.euler.r[2]);
  
  v_F_gear = wheelsys.getForces();
  v_M_gear = wheelsys.getMoments();
}


/**
        X       Aerodynamic force, lbs, in X-axis (+ forward)
        Y       Aerodynamic force, lbs, in Y-axis (+ right)
        Z       Aerodynamic force, lbs, in Z-axis (+ down)
        L       Aero. moment about X-axis (+ roll right), ft-lbs
        M       Aero. moment about Y-axis (+ pitch up), ft-lbs
        N       Aero. moment about Z-axis (+ nose right), ft-lbs

        0       Subscript implying initial, or nominal, value
        u       X-axis component of airspeed (ft/sec) (+ forward)
        v       Y-axis component of airspeed (ft/sec) (+ right)
        w       Z-axis component of airspeed (ft/sec) (+ down)
        p       X-axis ang. rate (rad/sec) (+ roll right), rad/sec
        q       Y-axis ang. rate (rad/sec) (+ pitch up), rad/sec
        r       Z-axis ang. rate (rad/sec) (+ yaw right), rad/sec
        beta    Angle of sideslip, degrees (+ wind in RIGHT ear)
        da      Aileron deflection, degrees (+ left ail. TE down)
        de      Elevator deflection, degrees (+ trailing edge down)
        dr      Rudder deflection, degrees (+ trailing edge LEFT)
 */
void CRRC_AirplaneSim_002::aero(SCALAR dt, TSimInputs* inputs) 
  // Calculate forces and moments for the current time step.  If Initialize is
  // zero, then re-initialize coefficients by reading in the coefficient file.
{
  SCALAR elevator, aileron, rudder;

  SCALAR Phat, Qhat, Rhat;  // dimensionless rotation rates
  SCALAR CL_left, CL_cent, CL_right; // CL values across the span
  SCALAR dCL_left,dCL_cent,dCL_right; // stall-induced CL changes
  SCALAR dCD_left,dCD_cent,dCD_right; // stall-induced CD changes
  SCALAR dCl,dCn,dCl_stall,dCn_stall;  // stall-induced roll,yaw moments
  SCALAR dCm_stall;  // Stall induced pitching moment.
  SCALAR CL_wing, CD_all, CD_scaled, Cl_w;
  SCALAR Cl_r_mod,Cn_p_mod;
  SCALAR CL,CD,Cl,Cm,Cn;
  SCALAR QS;
  
  CRRCMath::Vector3 C_xyz;

  CRRCMath::Vector3 v_R_omega_atmo;
  
  CRRCMath::Matrix33 G;
  CRRCMath::Matrix33 m_V_body;
  
  double V_rel_wind;
  double Alpha;
  double Beta;
    
  elevator = inputs->elevator;
  aileron  = inputs->aileron;
  rudder   = inputs->rudder;
  
  CRRCMath::Vector3 V_airmass_body  = eom.conv.body(v_V_local_airmass);

  // velocity of body relative to airmass
  v_V_body = eom.vel.val - V_airmass_body;

  V_rel_wind = v_V_body.length();

  /* Calculate alpha and beta */

  if (v_V_body.r[0] == 0)
    Alpha = 0;
  else
    Alpha = atan2( v_V_body.r[2], v_V_body.r[0] );

  if (V_rel_wind == 0)
    Beta = 0;
  else
    Beta = asin( v_V_body.r[1]/ V_rel_wind );

  
//  std::cout << Alpha << " ";
  
  // ---------------
  //
  double Sin_alpha = sin(Alpha);
  double Cos_alpha = cos(Alpha);
  double Cos_beta  = cos(Beta);

  /* compute gradients of Local velocities w.r.t. Body coordinates
      G_11  =  dU_local/dx_body
      G_12  =  dU_local/dy_body   etc.  */

  G = m_V_atmo_rwy * eom.conv.mat.trans();

  /* now compute gradients of Body velocities w.r.t. Body coordinates */
  /*  U_body_x  =  dU_body/dx_body   etc.  */

  m_V_body = eom.conv.mat * G;
  
  /* set rotation rates of airmass motion */
  v_R_omega_atmo.r[0] =  m_V_body.v[2][0];
  v_R_omega_atmo.r[1] = -m_V_body.v[2][1];
  v_R_omega_atmo.r[2] =  m_V_body.v[1][2];

  if (V_rel_wind != 0)
  {
    /* set net effective dimensionless rotation rates */
    Phat = (eom.angvel.val.r[0] - v_R_omega_atmo.r[0]) * B_ref / (2.0*V_rel_wind);
    Qhat = (eom.angvel.val.r[1] - v_R_omega_atmo.r[1]) * C_ref / (2.0*V_rel_wind);
    Rhat = (eom.angvel.val.r[2] - v_R_omega_atmo.r[2]) * B_ref / (2.0*V_rel_wind);
  }
  else
  {
    Phat=0;
    Qhat=0;
    Rhat=0;
  }

  /* compute local CL at three spanwise locations */
  CL_left  = CL_0 + CL_a*(Alpha - Alpha_0 - Phat*eta_loc);
  CL_cent  = CL_0 + CL_a*(Alpha - Alpha_0               );
  CL_right = CL_0 + CL_a*(Alpha - Alpha_0 + Phat*eta_loc);

  /* set CL-limit changes */
  dCL_left  = 0.;
  dCL_cent  = 0.;
  dCL_right = 0.;

  stalling=0;
  if (CL_left  > CL_max)
  {
    dCL_left  = CL_max-CL_left -CL_drop;
    stalling=1;
  }

  if (CL_cent  > CL_max)
  {
    dCL_cent  = CL_max-CL_cent -CL_drop;
    stalling=1;
  }

  if (CL_right > CL_max)
  {
    dCL_right = CL_max-CL_right -CL_drop;
    stalling=1;
  }

  if (CL_left  < CL_min)
  {
    dCL_left  = CL_min-CL_left -CL_drop;
    stalling=1;
  }

  if (CL_cent  < CL_min)
  {
    dCL_cent  = CL_min-CL_cent -CL_drop;
    stalling=1;
  }

  if (CL_right < CL_min)
  {
    dCL_right = CL_min-CL_right -CL_drop;
    stalling=1;
  }
  
//  std::cout << CL_left << " " << CL_cent << " " << CL_right << " ";

  /* set average wing CL */
  CL_wing = CL_0 + CL_a*(Alpha-Alpha_0)
    + 0.25*dCL_left + 0.5*dCL_cent + 0.25*dCL_right;

  /* correct profile CD for CL dependence and aileron dependence */
  CD_all = CD_prof
    + CD_CLsq *(CL_wing-CL_CD0)*(CL_wing-CL_CD0)
    + CD_AIsq * aileron        * aileron
    + CD_ELsq * elevator       * elevator;

  /* scale profile CD with Reynolds number via simple power law */
  if (V_rel_wind > 0.1)
  {
    CD_scaled = CD_all*pow(((double)V_rel_wind/(double)U_ref),Uexp_CD);
  }
  else
  {
    CD_scaled=CD_all;
  }

//  std::cout << CL_wing << " " << CD_scaled << " ";
  
  /* Scale lateral cross-coupling derivatives with wing CL */
  Cl_r_mod = Cl_r*CL_wing/CL_0;
  Cn_p_mod = Cn_p*CL_wing/CL_0;

  //std::cout << Cl_r_mod << " " << Cn_p_mod << " ";
  
  /* total average CD with induced and stall contributions */
  dCD_left  = CD_stall*dCL_left *dCL_left ;
  dCD_cent  = CD_stall*dCL_cent *dCL_cent ;
  dCD_right = CD_stall*dCL_right*dCL_right;

  //std::cout << dCD_left << " " << dCD_cent << " " << dCD_right << " ";
  
  /* total CL, with pitch rate and elevator contributions */
  CL = (CL_wing + CL_q*Qhat + CL_de*elevator)*Cos_alpha;

  //std::cout << CL << " ";
  
  /* assymetric stall will cause roll and yaw moments */
  dCl =  0.45*-1*(dCL_right-dCL_left)*0.5*eta_loc;
  dCn =  0.45*(dCD_right-dCD_left)*0.5*eta_loc;
  dCm_stall = (0.25*dCL_left + 0.5*dCL_cent + 0.25*dCL_right)*CG_arm;

  //std::cout << dCl << " " << dCn << " " << dCm_stall << " ";
  
  /* stall-caused moments in body axes */
  dCl_stall = dCl*Cos_alpha - dCn*Sin_alpha;
  dCn_stall = dCl*Sin_alpha + dCn*Cos_alpha;

  //std::cout << dCl_stall << " " << dCn_stall << " ";
  
  /* total CD, with induced and stall contributions */

  Cl_w = Cl_b*Beta  + Cl_p*Phat + Cl_r_mod*Rhat
    + dCl_stall  + Cl_da*aileron;
  CD = CD_scaled
    + (CL*CL + 32.0*Cl_w*Cl_w)*S_ref/(B_ref*B_ref*M_PI*span_eff)
      + 0.25*dCD_left + 0.5*dCD_cent + 0.25*dCD_right;

  //std::cout << Cl_w << " " << CD << " ";
  
  /* total forces in body axes */
  C_xyz.r[0] = -CD*Cos_alpha + CL*Sin_alpha*Cos_beta*Cos_beta;
  C_xyz.r[2] = -CD*Sin_alpha - CL*Cos_alpha*Cos_beta*Cos_beta;
  C_xyz.r[1] = CY_b*Beta  + CY_p*Phat + CY_r*Rhat + CY_dr*rudder;

  //std::cout << Cx << " " << Cz << " " << Cy << " ";

  /* total moments in body axes */
  Cl =        Cl_b*Beta  + Cl_p*Phat + Cl_r_mod*Rhat + Cl_dr*rudder
    + dCl_stall                               + Cl_da*aileron;
  Cn =        Cn_b*Beta  + Cn_p_mod*Phat + Cn_r*Rhat + Cn_dr*rudder
    + dCn_stall                               + Cn_da*aileron;
  Cm = Cm_0 + Cm_a*(Alpha-Alpha_0) +dCm_stall
    + Cm_q*Qhat                 + Cm_de*elevator;

  /* set dimensional forces and moments */
  {
    double DENSITY = env->GetRho(-eom.pos.val.r[2]);
    QS = 0.5*DENSITY*V_rel_wind*V_rel_wind * S_ref;
  }

//  std::cout << ">" << Cm_0 << " " << Cm_a << " " << Alpha_0 << " " << dCm_stall << " " << Cm_q << " " << Cm << "< ";
  
  // std::cout << Cl << " " << Cm << " " << Cn << " ";
  

  v_F_aero = C_xyz * QS;

  v_M_aero = CRRCMath::Vector3(Cl * QS * B_ref, Cm * QS * C_ref, Cn * QS * B_ref); 
  
  /*
  std::cout << (Cx * QS) << " " << (Cy * QS) << " " << (Cz * QS) << " " 
            << (Cl * QS * B_ref) << " " << (Cm * QS * C_ref) << " " << (Cn * QS * B_ref) << "\n";
   */



//  std::cout << "\n";
  
#if (EOM_TEST == 1)
  {
    double ele = 1.8*(-inputs->elevator-eom.angvel.val.r[1]);
    double ail = 1.8*(inputs->aileron-eom.angvel.val.r[0]);
    double rud = 1.8*(0.5*inputs->aileron-eom.angvel.val.r[2]);
   
    v_F_aero = CRRCMath::Vector3(4*inputs->throttle-0.8*eom.vel.val.r[0], -2.8*eom.vel.val.r[1], -2.8*eom.vel.val.r[2]);
    v_M_aero = CRRCMath::Vector3(ail, ele, rud);
    stalling = 0;
  }
#endif  
    
//  eom.pos.val.print("", "\n");
  
  
//  std::cout << "|V|=" << V_rel_wind << ", ";
//  std::cout << "alpha=" << Alpha << ", ";
//  std::cout << "beta=" << Beta << ", ";
//  V_body.print("V=", ", ");
//  v_F_aero.print("f=", ", ");
//  v_M_aero.print("m=", "\n");
}


void CRRC_AirplaneSim_002::aero_init() 
{
  stalling = 0;
  v_F_aero = CRRCMath::Vector3();
  v_M_aero = CRRCMath::Vector3();
}


double CRRC_AirplaneSim_002::getPropFreq()
{
  return(power->getPropFreq());
};
