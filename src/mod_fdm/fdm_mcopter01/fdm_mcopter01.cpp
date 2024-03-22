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
#include "fdm_mcopter01.h"

#include <math.h>
#include <iostream>

#include "../../mod_misc/ls_constants.h"
#include "../ls_geodesy.h"
#include "../../mod_misc/SimpleXMLTransfer.h"
#include "../../mod_misc/lib_conversions.h"
#include "../xmlmodelfile.h"

#define PITCH_FIXED_PITCH          1.0


Propdata::Propdata(SimpleXMLTransfer* cfg)
{
  x = cfg->getDouble("x");
  y = cfg->getDouble("y");
  mul_r = cfg->getInt("dir", 1);
}

void CRRC_AirplaneSim_MCopter01::initAirplaneState(double dRelVel,
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

  for (unsigned int n=0; n<controllers.size(); n++)
    controllers[n]->Reset();
  
  for (unsigned int n=0; n<power.size(); n++)
    power[n]->InitStates(CRRCMath::Vector3());
  
  ls_step_init();
}


void CRRC_AirplaneSim_MCopter01::update(TSimInputs* inputs,
                                        double      dt,
                                        int         multiloop) 
{
  CRRCMath::Vector3 v_V_local_airmass;
  CRRCMath::Vector3 v_V_gust_body = CRRCMath::Vector3();
  
  env->CalculateWind(v_P_CG_Rwy.r[0],        v_P_CG_Rwy.r[1],        v_P_CG_Rwy.r[2],
                     v_V_local_airmass.r[0], v_V_local_airmass.r[1], v_V_local_airmass.r[2]);
  
  CRRCMath::Vector3 v_F_aero, v_F_engine, v_F_gear; // Force x/y/z
  CRRCMath::Vector3 v_M_aero, v_M_engine, v_M_gear; // l/m/n <-> roll/pitch/yaw
  
  TSimInputs        OutputOfLocalControllers;
  CRRCMath::Vector3 v_URel;
  
  for (int n=0; n<multiloop; n++)
  {
    ls_step( dt );
    ls_aux(v_V_local_airmass, v_V_gust_body);

    // Global controllers first...
    env->ControllerCallback(dt, this, inputs, &myInputs);
    // ...local ones afterwards. aileron/elevator/rudder is output for rotation about x,y,z.
    OutputOfLocalControllers.CopyFrom(&myInputs); // in case there is no controller for something
    if (myInputs.throttle > 0.05)
    {
      for (unsigned int n=0; n<controllers.size(); n++)
        controllers[n]->Calc(dt, this, &myInputs, &OutputOfLocalControllers);
      v_URel = CRRCMath::Vector3(OutputOfLocalControllers.aileron, OutputOfLocalControllers.elevator, OutputOfLocalControllers.rudder);
    }
    else
    {
      v_URel = CRRCMath::Vector3();
      for (unsigned int n=0; n<controllers.size(); n++)
        controllers[n]->Reset();
    }
    // Because local controllers only remove key presses from the queue in myInputs and not from inputs,
    // it needs to be done manually here:
    inputs->ClearKeys();

    aero(dt, v_F_aero, v_M_aero);
    
    engine(dt, &OutputOfLocalControllers, v_URel, v_F_engine, v_M_engine);
    gear(&myInputs, v_F_gear, v_M_gear);
        
    ls_accel(v_F_aero + v_F_engine + v_F_gear, v_M_aero + v_M_engine + v_M_gear);        
  }
}


CRRC_AirplaneSim_MCopter01::CRRC_AirplaneSim_MCopter01(const char* filename, FDMEnviroment* myEnv, SimpleXMLTransfer* cfg) : EOM01("fdm_mcopter01.dat", myEnv)
{
  // Previously there has been code to load an old-style .air-file. 
  // This has been removed as CRRCSim includes an automatic converter.
  SimpleXMLTransfer* fileinmemory = new SimpleXMLTransfer(filename);

  power.clear();
  LoadFromXML(fileinmemory, cfg->getInt("airplane.verbosity", 5));
  InitStates();
  
  delete fileinmemory;
}


CRRC_AirplaneSim_MCopter01::CRRC_AirplaneSim_MCopter01(SimpleXMLTransfer* xml, FDMEnviroment* myEnv, SimpleXMLTransfer* cfg) : EOM01("fdm_mcopter01.dat", myEnv)
{
  power.clear();
  LoadFromXML(xml, cfg->getInt("airplane.verbosity", 5));
  InitStates();
}

void CRRC_AirplaneSim_MCopter01::LoadFromXML(SimpleXMLTransfer* xml, int nVerbosity)
{
  if (xml->getString("type").compare("mcopter01") != 0 ||
      xml->getInt("version") != 1)
  {
    throw XMLException("file is not for mcopter01");
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
    speed_damp       = cfg->getDouble("aero.speed.damp");
    roll_damp1       = cfg->getDouble("aero.roll.damp1", 0);
    yaw_damp1        = cfg->getDouble("aero.yaw.damp1",  0);    
    roll_damp2       = cfg->getDouble("aero.roll.damp2", 0);    
    yaw_damp2        = cfg->getDouble("aero.yaw.damp2",  0);    
        
    yaw_dist         = cfg->getDouble("aero.yaw.dist", 0);    
    roll_dist        = cfg->getDouble("aero.roll.dist", 0);
    pitch_dist       = cfg->getDouble("aero.pitch.dist", roll_dist);
    
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

  wheels.init(xml, 0);
  dRotorRadius = wheels.getWingspan()*0.5;
  dRotorZ      = wheels.getZHigh();

  props.clear();
  i = cfg->getChild("aero.props");
  for (int n=0; n<i->getChildCount(); n++)
    props.push_back(Propdata(i->getChildAt(n)));

  if (power.size() == 0)
  {    
    for (unsigned int n=0; n<props.size(); n++)
      power.push_back(new Power::Power(cfg, nVerbosity));
    dURef = 0.7 * cfg->getDouble("power.battery.U_0");
  }
  else
  {
    for (unsigned int n=0; n<power.size(); n++)
      power[n]->ReloadParams(cfg, nVerbosity);
  }
  
  controllers.clear();  
  Controller::LoadList(cfg->getChild("controllers"), controllers);  
}

void CRRC_AirplaneSim_MCopter01::InitStates()
{
  for (unsigned int n=0; n<power.size(); n++)
    power[n]->InitStates(CRRCMath::Vector3());
  filt_rnd_yaw.init(0);
  filt_rnd_roll.init(0);
  filt_rnd_pitch.init(0);
  dist_t = 0;
}

CRRC_AirplaneSim_MCopter01::~CRRC_AirplaneSim_MCopter01()
{
 for (unsigned int n=0; n<power.size(); n++)
    delete power[n];
}

/** \brief Calculate influence of gear and hardpoints
 *
 *  This method calculates the forces and moments on
 *  the airplane's body caused by the gear or other
 *  hardpoints touching the ground.
 *
 *  \param inputs   Current control inputs
 */
void CRRC_AirplaneSim_MCopter01::gear(TSimInputs* inputs, CRRCMath::Vector3& v_F, CRRCMath::Vector3& v_M)
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

float CRRC_AirplaneSim_MCopter01::GroundEffect(float dRotorToGround)
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


void CRRC_AirplaneSim_MCopter01::engine(SCALAR             dt,
                                        TSimInputs*        inputs,
                                        CRRCMath::Vector3  v_URel,
                                        CRRCMath::Vector3& v_F,
                                        CRRCMath::Vector3& v_M)
{
  CRRCMath::Vector3 F, M;
  
  v_F = CRRCMath::Vector3();
  v_M = CRRCMath::Vector3();

  inputs->pitch = PITCH_FIXED_PITCH;
  double thr_in = inputs->throttle;
  
  for (unsigned int n=0; n<power.size(); n++)
  {
    double x = props[n].x;
    double y = props[n].y;
    double l = sqrt(x*x + y*y);
    double mul_p = y/l;
    double mul_q = x/l;
    
    double thr = thr_in
      + v_URel.r[0] * mul_p
      + v_URel.r[1] * mul_q
      + v_URel.r[2] * props[n].mul_r;
    
    if (thr < 0)
      thr = 0;
    
    // Try to behave independent of battery voltage:
    inputs->throttle = thr * dURef/power[n]->GetVoltageAvg();
    
    // Die Reglerverstärkung ist:
    //   UDiff = omega_diff * kp * dURef
    // Der Integrator gibt nach der Zeit t mit der Regelabweichung omega die Spannung
    //   UDiff = t * omega * ki * dURef
    // aus.
    
    F = CRRCMath::Vector3();
    M = CRRCMath::Vector3();
    power[n]->step(dt, inputs, 
                   CRRCMath::Vector3(-v_V_wind_body.r[2],
                                     v_V_wind_body.r[1],
                                     v_V_wind_body.r[0]
                                     )*FT_TO_M,
                   &F, &M);
    
    v_F += CRRCMath::Vector3(0, 0, -F.r[0]);    
    v_M += CRRCMath::Vector3(y*F.r[0], x*F.r[0], -M.r[0] * props[n].mul_r);
    
    //std::cout << inputs->throttle << " " << power[n]->getPropFreq() << " ";
  }
  //std::cout << "\n";
    
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
void CRRC_AirplaneSim_MCopter01::aero(double dt, CRRCMath::Vector3& v_F, CRRCMath::Vector3& v_M)
{
  // Ground effect, see 'ground effect' in engine().
  double dGEMul = 1 + dGEDistMul * (GroundEffect(Altitude - dRotorZ) - 1);
  
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
  
  v_M = CRRCMath::Vector3(roll_dist  * filt_rnd_roll.val  * dGEMul - roll_damp1 * v_R_omega_body.r[0] - roll_damp2 * v_R_omega_body.r[0] * fabs(v_R_omega_body.r[0]),
                          pitch_dist * filt_rnd_pitch.val * dGEMul - roll_damp1 * v_R_omega_body.r[1] - roll_damp2 * v_R_omega_body.r[1] * fabs(v_R_omega_body.r[1]),
                          yaw_dist   * filt_rnd_yaw.val   * dGEMul - yaw_damp1  * v_R_omega_body.r[2] - yaw_damp2  * v_R_omega_body.r[2] * fabs(v_R_omega_body.r[2]));

  // Convert SI to that other buggy system.
  v_F *= N_TO_LBF;
  v_M *= NM_TO_LBFFT;    
}


void CRRC_AirplaneSim_MCopter01::ls_step_init() 
{
  CRRCMath::Vector3 v_F_aero, v_F_engine, v_F_gear; // Force x/y/z
  CRRCMath::Vector3 v_M_aero, v_M_engine, v_M_gear; // l/m/n <-> roll/pitch/yaw
  
  TSimInputs ZeroInput = TSimInputs();
  
  EOM01::ls_step_init();
  
  v_V_local = CRRCMath::Vector3();
  
  /*    Initialize vehicle model                        */
  ls_aux(CRRCMath::Vector3(), CRRCMath::Vector3());

  aero(0, v_F_aero, v_M_aero);
  gear(&ZeroInput, v_F_gear, v_M_gear);
  
  /*    Calculate initial accelerations */
  ls_accel(v_F_aero + v_F_gear, v_M_aero + v_M_gear);

  /* Initialize auxiliary variables */
  ls_aux(CRRCMath::Vector3(), CRRCMath::Vector3());
}

double CRRC_AirplaneSim_MCopter01::getPropFreq() 
{
  double max = power[0]->getPropFreq();
  for (unsigned int n=1; n<power.size(); n++)
    if (max < power[n]->getPropFreq())
      max = power[n]->getPropFreq();
  return(max);
}

int CRRC_AirplaneSim_MCopter01::ReloadParams(SimpleXMLTransfer* xml,
                                          SimpleXMLTransfer* cfg)
{
  LoadFromXML(xml, cfg->getInt("airplane.verbosity", 5));
  
  return(1);
}
