// -*- mode: c; mode: fold -*-
/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2008, 2009 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2008 - Jan Reucker
 * 
 * This file was copied from src/mod_fdm/fdm_larcsim/fdm_larcsim.cpp on 2008-09-05
 * and still contains some of its code.
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
#include "fdm_heli01.h"

#include <math.h>
#include <iostream>

#include "../../mod_misc/ls_constants.h"
#include "../ls_geodesy.h"
#include "../../mod_misc/SimpleXMLTransfer.h"
#include "../../mod_misc/lib_conversions.h"
#include "../xmlmodelfile.h"

#define PITCH_FIXED_PITCH          1.0
#define THROTTLE_COLLECTIVE_PITCH  1.0

/**
 * *****************************************************************************
 */

void CRRC_AirplaneSim_Heli01::initAirplaneState(double dRelVel,
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
  Phi       = dPhi;          // bank/roll angle  [rad]
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

  v_V_local.r[0] = 0;      // local x-velocity (north)   [ft/s]
  v_V_local.r[1] = 0;      // local y-velocity (east)    [ft/s]
  v_V_local.r[2] = 0;      // local z-velocity (down)     [ft/s]
    
  v_V_local_rel_ground.r[1] = v_V_local.r[1];
  
  v_R_omega_body    = CRRCMath::Vector3(R_X, R_Y, R_Z); // body rate   [rad/s]  
  v_V_dot_local     = CRRCMath::Vector3(); // local acceleration   [ft/s^2]

  dHeadingHoldInt = 0;
  
  power->InitStates(CRRCMath::Vector3());
  
  ls_step_init();
}


void CRRC_AirplaneSim_Heli01::update(TSimInputs* inputs,
                                     double      dt,
                                     int         multiloop) 
{
  CRRCMath::Vector3 v_V_local_airmass;
  CRRCMath::Vector3 v_V_gust_body = CRRCMath::Vector3();
  
  env->CalculateWind(v_P_CG_Rwy.r[0],        v_P_CG_Rwy.r[1],        v_P_CG_Rwy.r[2],
                     v_V_local_airmass.r[0], v_V_local_airmass.r[1], v_V_local_airmass.r[2]);
  
  CRRCMath::Vector3 v_F_aero, v_F_engine, v_F_gear; // Force x/y/z
  CRRCMath::Vector3 v_M_aero, v_M_engine, v_M_gear; // l/m/n <-> roll/pitch/yaw
  
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
            
    ls_step( dt );
    ls_aux(v_V_local_airmass, v_V_gust_body);

    env->ControllerCallback(dt, this, inputs, &myInputs);
    
    aero(dt, &myInputs, v_F_aero, v_M_aero);
    
#if FDM_LOG_AERO_OUT != 0
    logVal(v_F_aero);
    logVal(v_M_aero);
#endif
    
    engine(dt, &myInputs , v_F_engine, v_M_engine);
    gear(&myInputs, v_F_gear, v_M_gear);
        
    ls_accel(v_F_aero + v_F_engine + v_F_gear, v_M_aero + v_M_engine + v_M_gear,
             myInputs.heli_fixed_z, fFixedHorizon);
  }
}


CRRC_AirplaneSim_Heli01::CRRC_AirplaneSim_Heli01(const char* filename, FDMEnviroment* myEnv, SimpleXMLTransfer* cfg) : EOM01("fdm_heli01.dat", myEnv)
{
  // Previously there has been code to load an old-style .air-file. 
  // This has been removed as CRRCSim includes an automatic converter.
  SimpleXMLTransfer* fileinmemory = new SimpleXMLTransfer(filename);
  
  power = 0;
  LoadFromXML(fileinmemory, cfg->getInt("airplane.verbosity", 5));
  InitStates();
  
  delete fileinmemory;
}


CRRC_AirplaneSim_Heli01::CRRC_AirplaneSim_Heli01(SimpleXMLTransfer* xml, FDMEnviroment* myEnv, SimpleXMLTransfer* cfg) : EOM01("fdm_heli01.dat", myEnv)
{
  power = 0;
  LoadFromXML(xml, cfg->getInt("airplane.verbosity", 5));
  InitStates();
}

void CRRC_AirplaneSim_Heli01::LoadFromXML(SimpleXMLTransfer* xml, int nVerbosity)
{
  if (xml->getString("type").compare("heli01") != 0 ||
      xml->getInt("version") != 3)
  {
    throw XMLException("file is not for heli01");
  }    
    
  SimpleXMLTransfer* i;
  SimpleXMLTransfer* cfg = XMLModelFile::getConfig(xml);  
  
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
    
  {
    fCoaxial         =(cfg->getInt("aero.coaxial",      0) != 0);
    fFixedPitch      =(cfg->getInt("power.fixed_pitch", 0) != 0);
    speed_damp       = cfg->getDouble("aero.speed.damp");    
        
    yaw_ctrl         = cfg->getDouble("aero.yaw.ctrl");
    yaw_damp         = cfg->getDouble("aero.yaw.damp");    
    yaw_damp_min_rel = cfg->getDouble("aero.yaw.damp_min_rel");
    yaw_dist         = cfg->getDouble("aero.yaw.dist", 0);
    yaw_off          = cfg->getDouble("aero.yaw.off",  0);
    cp_to_yaw        = cfg->getDouble("aero.yaw.cp_to_yaw",   0);
    dHeadingHold     = cfg->getDouble("aero.yaw.HeadingHold", 0);
    
    roll_ctrl        = cfg->getDouble("aero.roll.ctrl");
    roll_damp        = cfg->getDouble("aero.roll.damp");    
    roll_dist        = cfg->getDouble("aero.roll.dist", 0);
    dForwardToRoll   = cfg->getDouble("aero.roll.ForwardToRoll", 1E-4);
    
    pitch_ctrl       = -1 * cfg->getDouble("aero.pitch.ctrl", roll_ctrl);
    pitch_damp       =      cfg->getDouble("aero.pitch.damp", roll_damp);
    pitch_dist       =      cfg->getDouble("aero.pitch.dist", roll_dist);

    cp_ctrl          = cfg->getDouble("aero.cp.ctrl", 1);
    cp_off           = cfg->getDouble("aero.cp.off",  0);

    yaw_moment_mul   = cfg->getDouble("aero.yaw.moment_mul", 1);
    
    if (fCoaxial)
    {
      yaw_moment_mul = 0;
      dForwardToRoll = 0;
      yaw_off        = 0;
      cp_to_yaw      = 0;
    }
     
    yaw_vane      = cfg->getDouble("aero.yaw.vane",   0);
    pitch_vane    = cfg->getDouble("aero.pitch.vane", 0);
    
    // convert to internal representation:
    yaw_damp   *= I_zz / KG_M_M_TO_SLUG_FT_FT;
    yaw_dist   *= I_zz / KG_M_M_TO_SLUG_FT_FT;
    yaw_vane   *= I_zz / KG_M_M_TO_SLUG_FT_FT;
    roll_damp  *= I_xx / KG_M_M_TO_SLUG_FT_FT;
    roll_dist  *= I_xx / KG_M_M_TO_SLUG_FT_FT;
    pitch_damp *= I_yy / KG_M_M_TO_SLUG_FT_FT;
    pitch_dist *= I_yy / KG_M_M_TO_SLUG_FT_FT;
    pitch_vane *= I_yy / KG_M_M_TO_SLUG_FT_FT;

    roll_ctrl  *= roll_ctrl  * roll_damp /0.5;
    pitch_ctrl *= pitch_ctrl * pitch_damp/0.5;
    
    if (fabs(dHeadingHold) > 1.0E-8)
    {
      dHeadingHold *= I_zz / KG_M_M_TO_SLUG_FT_FT;
      yaw_ctrl *= 2;
    }
    else
    {
      yaw_ctrl *= yaw_damp_min_rel * yaw_damp/0.5;
    }
    
    // The ground effect parameters should be quite independent of the helicopter
    // parameters...shouldn't they? However, they can be adjusted.
    dGEDistMul  = xml->getDouble("GroundEffect.dist.mul",  1.5);

    {
      double tau  = xml->getDouble("Disturbance.tau_filter", 0.2);
      dist_t_init = xml->getDouble("Disturbance.time",       0.2);

      filt_rnd_yaw.SetTau(tau);
      filt_rnd_roll.SetTau(tau);
      filt_rnd_pitch.SetTau(tau);
    }
  }  
  
  wheels.init(xml, 0);
  dRotorRadius = wheels.getWingspan()*0.5;
  dRotorZ      = wheels.getZHigh();

  if (power == 0)
    power = new Power::Power(cfg, nVerbosity);
  else
    power->ReloadParams(cfg, nVerbosity);
  
  
  if (cfg->getInt("aero.cp.auto_off", 1) != 0)
  {
    // Adjust cp.off, so that the heli stays level with max. ground effect
    // and a certain throttle/cp-command applied. I don't want the model-creator
    // to adjust cp.off manually, because that's tiresome...
    Power::Power* lp = new Power::Power(cfg, 0);
    double FLevel = LBF_TO_N * Mass * env->GetG(0) / ( GroundEffect(wheels.getZLow() - dRotorZ) * 1.03 );
    CRRCMath::Vector3 torque;
    if (fFixedPitch)
    {
      // ask the power system about the throttle input needed to stay level
      cp_off = lp->Sim_GetThrottle(CRRCMath::Vector3(), FLevel, PITCH_FIXED_PITCH, torque);
    }
    else
    {
      // ask the power system about the pitch input needed to stay level
      cp_off = lp->Sim_GetPitch(CRRCMath::Vector3(), FLevel, THROTTLE_COLLECTIVE_PITCH, torque);
    }
    
    std::cout << "Automagically set config.aero.cp.off=" << cp_off << "\n";
    delete lp;
  }  
  
  if (cfg->getInt("aero.yaw.torque_auto", 1) != 0 && fCoaxial == false && fabs(dHeadingHold) < 1.0E-8)
  {
    // Automatically adjust torque of main rotor, mixer to tail rotor etc.
    Power::Power* lp = new Power::Power(cfg, 0);
    
    // Hovering out of ground effect should be compensated:
    double FLevel = LBF_TO_N * Mass * env->GetG(5*dRotorRadius);
    CRRCMath::Vector3 torqueA;
    CRRCMath::Vector3 torqueB;
    TSimInputs inp;
    float throttle_usrA;
    float throttle_usrB;
    
    if (fFixedPitch)
    {
      inp.throttle  = lp->Sim_GetThrottle(CRRCMath::Vector3(), FLevel, PITCH_FIXED_PITCH, torqueA);
      throttle_usrA = (inp.throttle - cp_off) / (1-cp_off) * 0.5;

      float maxthrottle = 1.0;
      inp.throttle  = (maxthrottle + inp.throttle) / 2;
      inp.pitch     = PITCH_FIXED_PITCH;
      throttle_usrB = (inp.throttle - cp_off) / (1-cp_off) * 0.5;
    }
    else
    {
      inp.pitch     = lp->Sim_GetPitch(CRRCMath::Vector3(), FLevel, THROTTLE_COLLECTIVE_PITCH, torqueA);
      throttle_usrA = (inp.pitch - cp_off) / cp_ctrl;

      float maxpitch = cp_ctrl * 0.5 + cp_off;
      inp.pitch     = (maxpitch + inp.pitch) / 2;
      inp.throttle  = THROTTLE_COLLECTIVE_PITCH;
      throttle_usrB = (inp.pitch - cp_off) / cp_ctrl;
    }
    
    
    // Climbing with stick halfway between hover and full should be compensated:
    {
      CRRCMath::Vector3 force;
      float V_X = 0;
      float V_X_n;
      float force_res;
      
      do
      {
        lp->Sim_UntilStable(&inp, CRRCMath::Vector3(V_X, 0, 0), 0.0001, &force, &torqueB);
        force_res = force.length() - FLevel;
        
        // std::cout << V_X << " " << force_res << " " << (V_X*V_X*speed_damp) << "\n";
                
        V_X_n = sqrt(force_res / speed_damp);
        V_X = (V_X_n - V_X) * 0.1 + V_X;        
      }
      while ( fabs(V_X_n - V_X) / V_X > 0.001 );
    }
    
    // equation from aero():
    // note: -0.5 has been left out, throttle_usrX has been calculated accordingly
    //   yc = yaw_off + cp_to_yaw * throttle_usr
    // here:
    //   torque.r[0] * yaw_moment_mul = yaw_off + cp_to_yaw * throttle_usr
    torqueA.r[0] *= yaw_moment_mul;
    torqueB.r[0] *= yaw_moment_mul;
    //   torque.r[0] = yaw_off + cp_to_yaw * throttle_usr
    float f = throttle_usrB / throttle_usrA;
    yaw_off   = (torqueB.r[0] - torqueA.r[0] * f) / (1 - f);
    cp_to_yaw = (torqueA.r[0] - yaw_off) / throttle_usrA;

    std::cout << "Automagically set config.aero.yaw.off      =" << yaw_off   << "\n";
    std::cout << "Automagically set config.aero.yaw.cp_to_yaw=" << cp_to_yaw << "\n";
    
    delete lp;
  }  
  
  fFixedHorizon = false;
  switch (cfg->getInt("adjust", 0))
  {
    case 1:
      MulM = CRRCMath::Vector3(0, 0, 0);
      MulF = CRRCMath::Vector3(0, 0, 1);
      fFixedHorizon = true;
      break;

    case 2: // test rotor torque
      MulM = CRRCMath::Vector3(1, 1, 1);
      MulF = CRRCMath::Vector3(0, 0, 1);
      yaw_dist = 0;
      fFixedHorizon = true;
      break;
      
    case 3:
      MulM = CRRCMath::Vector3(1, 1, 1);
      MulF = CRRCMath::Vector3(0, 0, 0);
      yaw_dist       = 0;
      roll_dist      = 0;
      pitch_dist     = 0;
      yaw_moment_mul = 0;
      yaw_off        = 0;
      cp_to_yaw      = 0;
      yaw_vane       = 0;
      pitch_vane     = 0;
      break;

    case 4:
      MulM = CRRCMath::Vector3(1, 1, 1);
      MulF = CRRCMath::Vector3(0, 0, 0);
      yaw_moment_mul = 0;
      yaw_off        = 0;
      cp_to_yaw      = 0;
      yaw_vane       = 0;
      pitch_vane     = 0;
      break;
      
    default:
      MulM = CRRCMath::Vector3(1, 1, 1);
      MulF = CRRCMath::Vector3(1, 1, 1);
      break;
  }
}

void CRRC_AirplaneSim_Heli01::InitStates()
{
  filt_rnd_yaw.init(0);
  filt_rnd_roll.init(0);
  filt_rnd_pitch.init(0);
  dist_t = 0;
}

CRRC_AirplaneSim_Heli01::~CRRC_AirplaneSim_Heli01()
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
void CRRC_AirplaneSim_Heli01::gear(TSimInputs* inputs, CRRCMath::Vector3& v_F, CRRCMath::Vector3& v_M)
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

float CRRC_AirplaneSim_Heli01::GroundEffect(float dRotorToGround)
{
  // There is a graph on
  //   http://user.cs.tu-berlin.de/~calle/marvin/dissertation/aerodynamik.html
  // which shows (approximately)
  //   1 / (1 + 1.0/( a+b*dDistRel+c*(dDistRel*dDistRel)))
  float dRotorToGear = wheels.getZLow() - dRotorZ;
  if (dRotorToGround < dRotorToGear)
    dRotorToGround = dRotorToGear;
  float dDistRel = dRotorToGround / (2*dRotorRadius);
  float a =   3.76509;
  float b = -27.6284;
  float c = 103.083;
  return(1 + 1.0/( a+b*dDistRel+c*(dDistRel*dDistRel)) );
}


void CRRC_AirplaneSim_Heli01::engine( SCALAR dt, TSimInputs* inputs, CRRCMath::Vector3& v_F, CRRCMath::Vector3& v_M)
{
  v_F = CRRCMath::Vector3();
  v_M = CRRCMath::Vector3();
  
  if (fFixedPitch)
  {
    double thr = inputs->throttle;
    
    // scale throttle:
    //   in     out
    //   0.0    0.0
    //   0.5    cp_off
    //   1.0    1.0
    if (thr < 0.5)
      inputs->throttle = cp_off * thr/0.5;
    else
      inputs->throttle = cp_off + (1.0-cp_off) * (thr-0.5)/0.5;
    
    inputs->pitch = PITCH_FIXED_PITCH;
  }
  else
  {
    inputs->pitch    = cp_ctrl * (inputs->throttle - 0.5) + cp_off;
    inputs->throttle = THROTTLE_COLLECTIVE_PITCH;
  }
  
  power->step(dt, inputs, 
              CRRCMath::Vector3(-v_V_wind_body.r[2],
                                v_V_wind_body.r[1],
                                v_V_wind_body.r[0]
                                )*FT_TO_M,
              &v_F, &v_M);
  
  v_M = CRRCMath::Vector3(0, 0, -v_M.r[0] * yaw_moment_mul);

  v_F = CRRCMath::Vector3(0, 0, -v_F.r[0]);
  
  // --- Ground effect -------------------
  double dGEMul = GroundEffect(Altitude - dRotorZ);
  // Transform from body to local frame
  v_F = LocalToBody.multrans(v_F);
  // apply
  if (v_F.r[2] < 0)
    v_F.r[2] *= dGEMul;
  // Transform from local to body frame
  v_F = LocalToBody * v_F;

  // Convert SI to that other buggy system.
  v_F *= N_TO_LBF;
  v_M *= NM_TO_LBFFT;  
}


// Calculate forces and moments for the current time step.
void CRRC_AirplaneSim_Heli01::aero(double dt, TSimInputs* inputs, CRRCMath::Vector3& v_F, CRRCMath::Vector3& v_M)
{
  // Ground effect, see 'ground effect' in engine().
  double dGEMul = 1 + dGEDistMul * (GroundEffect(Altitude - dRotorZ) - 1);
  
  CRRCMath::Vector3 v_V_wind_body_SI    = v_V_wind_body * FT_TO_M;
  // some forces and moments are proportional to v^2:
  CRRCMath::Vector3 v_V_wind_body_SI_sq = v_V_wind_body * v_V_wind_body.length() * FT_TO_M * FT_TO_M;
  
  v_F = v_V_wind_body_SI_sq * (-speed_damp);
  
  dist_t -= dt;
  if (dist_t < 0)
  {
    dist_t += dist_t_init;
    in_rnd_yaw   = rnd_yaw.Get();
    in_rnd_roll  = rnd_roll.Get();
    in_rnd_pitch = rnd_pitch.Get();
  }
  
  filt_rnd_yaw.step(dt, in_rnd_yaw);
  filt_rnd_roll.step(dt, in_rnd_roll);
  filt_rnd_pitch.step(dt, in_rnd_pitch);
  
  double yd = yaw_damp * (1 - 2*fabs(inputs->rudder)*(1 - yaw_damp_min_rel));
  
  double yc;
  if (fabs(dHeadingHold) > 1.0E-8)
  {
    dHeadingHoldInt += dHeadingHold*(-yaw_ctrl * inputs->rudder - v_R_omega_body.r[2]);
    yc = dHeadingHoldInt;
  }
  else
    yc = yaw_off - yaw_ctrl * inputs->rudder + cp_to_yaw * (inputs->throttle - 0.5);
  
  v_M = CRRCMath::Vector3(  roll_ctrl  * inputs->aileron
                          - roll_damp  * v_R_omega_body.r[0] * fabs(v_R_omega_body.r[0])
                          + roll_dist  * filt_rnd_roll.val * dGEMul,
                          - pitch_ctrl * inputs->elevator
                          - pitch_damp * v_R_omega_body.r[1] * fabs(v_R_omega_body.r[1])
                          + pitch_dist * filt_rnd_pitch.val * dGEMul,
                          yc
                          - yd         * v_R_omega_body.r[2]           // yaw damping is linear (gyro!)
                          + yaw_dist   * filt_rnd_yaw.val * dGEMul);

  // When not hovering and not being a coaxial rotor, relative wind velocity adds 
  // differently to the blades veloctiy, and creates a moment normal to moving
  // direction.
  // Project relative wind velocity onto rotor disc: we already have that in
  // v_V_wind_body. So just apply this:
  v_M += CRRCMath::Vector3(v_V_wind_body_SI.r[0] * dForwardToRoll,
                           v_V_wind_body_SI.r[1] * dForwardToRoll,
                           0);
  
  // vane effect
  v_M += CRRCMath::Vector3(0,
                           -pitch_vane * v_V_wind_body_SI_sq.r[2],
                           yaw_vane    * v_V_wind_body_SI_sq.r[1]);
    
  // Convert SI to that other buggy system.
  v_F *= N_TO_LBF;
  v_M *= NM_TO_LBFFT;    
}


void CRRC_AirplaneSim_Heli01::ls_step_init() 
{
  CRRCMath::Vector3 v_F_aero, v_F_engine, v_F_gear; // Force x/y/z
  CRRCMath::Vector3 v_M_aero, v_M_engine, v_M_gear; // l/m/n <-> roll/pitch/yaw
  
  TSimInputs ZeroInput = TSimInputs();
  
  EOM01::ls_step_init();
    
  /*    Initialize vehicle model                        */
  ls_aux(CRRCMath::Vector3(), CRRCMath::Vector3());

  aero(0, &ZeroInput, v_F_aero, v_M_aero);
  engine(0, &ZeroInput, v_F_engine, v_M_engine);
  gear(&ZeroInput, v_F_gear, v_M_gear);

  /*    Calculate initial accelerations */
  ls_accel(v_F_aero + v_F_engine + v_F_gear, v_M_aero + v_M_engine + v_M_gear, 
           EOM01_FIXED_Z_OFF, fFixedHorizon);

  /* Initialize auxiliary variables */
  ls_aux(CRRCMath::Vector3(), CRRCMath::Vector3());
}

double CRRC_AirplaneSim_Heli01::getPropFreq() 
{
  return(power->getPropFreq());
}

int CRRC_AirplaneSim_Heli01::ReloadParams(SimpleXMLTransfer* xml,
                                          SimpleXMLTransfer* cfg)
{
  LoadFromXML(xml, cfg->getInt("airplane.verbosity", 5));
  
  return(1);
}
