/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2008 - Jan Reucker (refactoring, see note below)
 *
 * This file is partially based on work by
 *   Jan Kansky
 *   Jens Wilhelm Wulf
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
/** \file gear.cpp
 *
 *  Definitions of class methods for hardpoint/wheel management
 */

#include "gear.h"
#include <stdexcept>
#include "../../mod_misc/ls_constants.h"
#include "../xmlmodelfile.h"
#include "../../mod_main/EventDispatcher.h"


/**
 * Create a HardPointRotation
 *
 * \param xml <animation> part of the model file that contains the
 *            description of the animation
 */
HardPointRotation::HardPointRotation(SimpleXMLTransfer *xml, TSimInputs const& in)
{
  bool failed = false;
  
  // evaluate <object> tag
  SimpleXMLTransfer *map = xml->getChild("object", true);
  symbolic_name = map->getString("name", "no_name_set");
  max_angle_rad = (float)map->getDouble("max_angle", 0.0);
  abs_max_angle_rad = (float)fabs((double)max_angle_rad);

  // find hinges and evaluate all <control> tags
  int num_controls = 0;
  int num_hinges = 0;
  for (int i = 0; i < xml->getChildCount(); i++)
  {
    SimpleXMLTransfer *child = xml->getChildAt(i);
    if (child->getName() == "hinge")
    {
      // found a <hinge> child
      CRRCMath::Vector3 pos;
      pos.r[0] = (float)(child->getDouble("x", 0.0));
      pos.r[1] = (float)(child->getDouble("y", 0.0));
      pos.r[2] = (float)(child->getDouble("z", 0.0));
      if (num_hinges < 2)
      {
        hinge[num_hinges] = pos;
      }
      num_hinges++;
    }
    else if (child->getName() == "control")
    {
      // found a <control> child
      // The "*2" factor for each gain value scales the control input
      // values from -0.5...+0.5 to -1.0...+1.0. This saves one
      // float multiplication per mapping in the runtime update() routine.
      // NB: unsigned control (throttle, spoiler, retract) are not scaled,
      //     since control input is already in range 0.0...+1.0
      std::string mapping = child->getString("mapping", "NOTHING");
      float gain = (float)child->getDouble("gain", 1.0);
      std::cout << "  mapped to " << mapping << " with gain " << gain;
      std::cout << " and max_angle_rad " << max_angle_rad << std::endl;
      if (mapping == "ELEVATOR")
      {
        datasource.push_back(&in.elevator);
        source_gain.push_back(gain * 2);
        num_controls++;
      }
      else if (mapping == "AILERON")
      {
        datasource.push_back(&in.aileron);
        source_gain.push_back(gain * 2);
        num_controls++;
      }
      else if (mapping == "THROTTLE")
      {
        datasource.push_back(&in.throttle);
        source_gain.push_back(gain);
        num_controls++;
      }
      else if (mapping == "RUDDER")
      {
        datasource.push_back(&in.rudder);
        source_gain.push_back(gain * 2);
        num_controls++;
      }
      else if (mapping == "FLAP")
      {
        datasource.push_back(&in.flap);
        source_gain.push_back(gain * 2);
        num_controls++;
      }
      else if (mapping == "SPOILER")
      {
        datasource.push_back(&in.spoiler);
        source_gain.push_back(gain);
        num_controls++;
      }
      else if (mapping == "RETRACT")
      {
        datasource.push_back(&in.retract);
        source_gain.push_back(gain);
        num_controls++;
      }
      else if (mapping == "PITCH")
      {
        datasource.push_back(&in.pitch);
        source_gain.push_back(gain * 2);
        num_controls++;
      }
      else
      {
        fprintf(stderr, "HardPointRotation: ignoring <control> tag without mapping.\n");
      }
      
    }
  }

  if (num_controls < 1)
  {
    fprintf(stderr, "HardPointRotation: found animation without proper <control> tag. Animation disabled.\n");
    failed = true;
  }

  if (num_hinges < 2)
  {
    fprintf(stderr, "HardPointRotation: Must specify exactly two hinges!\n");
    failed = true;
  }
  else
  {
    if (num_hinges > 2)
    {
      fprintf(stderr, "HardPointRotation: Must specify exactly two hinges!\n");
      fprintf(stderr, "HardPointRotation: Ignoring excessive hinge tag(s).\n");
    }
    axis = hinge[1] - hinge[0];
    if (axis.length() < 0.001)
    {
      fprintf(stderr, "HardPointRotation: Insufficient spacing between hinges!\n");
      failed = true;
    }
  }

  if (failed)
  {
    fprintf(stderr, "HardPointRotation: Animation setup failed.\n");
    // set to some non-critical defaults
    datasource.resize(1);
    datasource[0] = &fallback_data;
    source_gain.resize(1);
    source_gain[0] = 1.0;
    hinge[0] = CRRCMath::Vector3(0.0, 0.0, 0.0);
    hinge[1] = CRRCMath::Vector3(1.0, 0.0, 0.0);
    axis = hinge[1] - hinge[0];
  }
  else
  {
    std::cerr << "HardPointRotation: set up animated hardpoint ";
    std::cerr << symbolic_name << std::endl;
  }
  
  move_orig.makeTranslation(hinge[0] * -1);
  move_back.makeTranslation(hinge[0]);

  //~ realInit();

}


void HardPointRotation::transform(CRRCMath::Vector3& hp)
{
  hp = xform * hp;
}

/** Update the transformation
 *
 * This method recalculates the transformation matrix
 * based on the current control inputs.
 */
void HardPointRotation::update()
{
  // calculate the deflection angle by summing up the contribution
  // of all mapped control inputs times the individual gain
  float angle = 0.0f;
  for (int i = 0; i < (int)datasource.size(); i++)
  {
    angle += *datasource[i] * source_gain[i] * max_angle_rad;
  }
  // limit to max_angle (caution: max_angle could be < 0.0, so we use the abs value!)
  if (angle > abs_max_angle_rad)
  {
    angle = abs_max_angle_rad;
  }
  else if (angle < -abs_max_angle_rad)
  {
    angle = -abs_max_angle_rad;
  }

  // calculate transformation matrix for surface rotation
  CRRCMath::Matrix44f rotation(axis, angle);

  xform = move_back * rotation * move_orig;
}

/**
 * Constructor for a wheel
 */
Wheel::Wheel(const WheelSystem* ws)
: myWheelSystem(ws)
{
}


/**
 * Interface to CRRC_AirplaneSim_Larcsim:
 *
 * CRRCMath::Matrix33 LocalToBody           (Transformation matrix local to body)
 * CRRCMath::Vector3  v_P_CG_Rwy            (CG relative to runway, in rwy coordinates N/E/D)
 * CRRCMath::Vector3  v_R_omega_body        (Angular body rates)
 * CRRCMath::Vector3  v_V_local_rel_ground  (V rel w.r.t. earth surface)
 * SCALAR             euler_angles_v[2]     (Psi)
 */
void Wheel::update( FDMEnviroment* env,
                    CRRCMath::Matrix33 LocalToBody,
                    CRRCMath::Vector3  const& v_P_CG_Rwy,
                    CRRCMath::Vector3  const& v_R_omega_body,
                    CRRCMath::Vector3  const& v_V_local_rel_ground,
                    SCALAR psi)
{
  /*
   * Constants & coefficients for tyres on tarmac - ref [1]
   */

  /* skid function looks like:
   *
   *           mu  ^
   *               |
   *       max_mu  |       +
   *               |      /|
   *   sliding_mu  |     / +------
   *               |    /
   *               |   /
   *               +--+------------------------>
   *               |  |    |      sideward V
   *               0 bkout skid
   *                V     V
   */

  const SCALAR sliding_mu   = 0.55;
  const SCALAR rolling_mu   = 0.01;
  const SCALAR max_brake_mu = 0.6;
  const SCALAR max_mu       = 0.8;
  const SCALAR bkout_v      = 0.1;
  const SCALAR skid_v       = 1.0;

  /*
   * Local data variables
   */

  SCALAR reaction_normal_force;   /* wheel normal (to rwy) force */
  SCALAR cos_wheel_hdg_angle, sin_wheel_hdg_angle;  /* temp storage */
  SCALAR steering_angle_rad;
  SCALAR v_wheel_forward, v_wheel_sideward,  abs_v_wheel_sideward;
  SCALAR forward_mu, sideward_mu; /* friction coefficients */
  SCALAR beta_mu;                 /* breakout friction slope */
  SCALAR forward_wheel_force, sideward_wheel_force;

  CRRCMath::Vector3 temp3a, temp3b;
  CRRCMath::Vector3 v_V_wheel_local;
  CRRCMath::Vector3 v_F_wheel_local;
  
  /* wheel offset from cg,  X-Y-Z */
  CRRCMath::Vector3 v_P_wheel_cg_body;
  
  /* wheel offset from cg,  N-E-D */
  CRRCMath::Vector3 v_P_wheel_cg_local;
  
  /* wheel offset from rwy, N-E-U */
  CRRCMath::Vector3 v_P_wheel_rwy_local;
  

  /* update the animation */
  if (hpt != NULL)
  {
    hpt->update();
  }

  beta_mu = max_mu/(skid_v-bkout_v);
  
  /*========================================*/
  /* Calculate wheel position w.r.t. runway */
  /*========================================*/

  /* First calculate wheel location w.r.t. cg in body (X-Y-Z) axes... */

  v_P_wheel_cg_body = v_P;

  /* ...transform the wheel position if it is coupled to a transformation... */
  if (hpt != NULL)
  {
    hpt->transform(v_P_wheel_cg_body);
  }

  /* then converting to local (North-East-Down) axes... */

  v_P_wheel_cg_local = LocalToBody.multrans(v_P_wheel_cg_body);
  
  /* Add wheel offset to cg location in local axes */

  v_P_wheel_rwy_local = v_P_wheel_cg_local + v_P_CG_Rwy;

  /*============================*/
  /* Calculate wheel velocities */
  /*============================*/

  /* contribution due to angular rates */

  temp3a = v_R_omega_body * v_P_wheel_cg_body;

  /* transform into local axes */

  temp3b = LocalToBody.multrans(temp3a);

  /* plus contribution due to cg velocities */

  v_V_wheel_local = temp3b + v_V_local_rel_ground;

  /*===========================================*/
  /* Calculate forces & moments for this wheel */
  /*===========================================*/

  /* Steering angle */
  /* First, determine the control input for this wheel */
  steering_angle_rad = myWheelSystem->wheel_inputs.GetInput(steering_mapping);
  /* Then calculate the real angle. Full control input (+-0.5)
     shall result in full wheel deflection  */
  steering_angle_rad = 2 * steering_angle_rad * steering_max_angle;

  /* Calculate sideward and forward velocities of the wheel
   in the runway plane     */
  SCALAR tmp_angle = caster_angle_rad + steering_angle_rad + psi;
  cos_wheel_hdg_angle = cos(tmp_angle);
  sin_wheel_hdg_angle = sin(tmp_angle);

  v_wheel_forward  = v_V_wheel_local.r[0]*cos_wheel_hdg_angle
                   + v_V_wheel_local.r[1]*sin_wheel_hdg_angle;
  v_wheel_sideward = v_V_wheel_local.r[1]*cos_wheel_hdg_angle
                   - v_V_wheel_local.r[0]*sin_wheel_hdg_angle;    

  /* Calculate normal load force (simple spring constant) */

  reaction_normal_force = 0.;

  SCALAR z_earth = -1*env->GetSceneryHeight(v_P_wheel_rwy_local.r[0], v_P_wheel_rwy_local.r[1]);
  
  if (v_P_wheel_rwy_local.r[2] > z_earth)
  {
    // Forces are in lbf here, lengths in ft, velocities in ft/s. 
    // So:
    // 1 slug * 1 ft / s^2 = spring_constant * 1 ft - 1 ft/s  * spring_damping
    // spring_constant  = slug / s^2 = lbf / ft
    // spring_damping   = slug / s   = lbf * s / ft
    reaction_normal_force = spring_constant*(z_earth-v_P_wheel_rwy_local.r[2])
                           - v_V_wheel_local.r[2]*spring_damping;
  }
  
  /* Crash detection. Normal force is negative. */
  if (-reaction_normal_force > max_force)
  {
    /* emit a crash event */
    CrashEvent ev;
    EventDispatcher::getInstance()->raise(&ev);
    std::cout << "Hardpoint " << nID << ": max_force exceeded (";
    std::cout << -reaction_normal_force << " lbf > " << max_force << " lbf)" << std::endl;
  }

  /* Calculate friction coefficients */
  forward_mu = (max_brake_mu - rolling_mu) * percent_brake + rolling_mu;
  abs_v_wheel_sideward = sqrt(v_wheel_sideward*v_wheel_sideward);
  sideward_mu = sliding_mu;
  if (abs_v_wheel_sideward < skid_v)
    sideward_mu = (abs_v_wheel_sideward - bkout_v)*beta_mu;
  if (abs_v_wheel_sideward < bkout_v)
    sideward_mu = 0.;

  /* Calculate foreward and sideward reaction forces */

  forward_wheel_force  =   forward_mu*reaction_normal_force;
  sideward_wheel_force =  sideward_mu*reaction_normal_force;
  if(v_wheel_forward < 0.)
    forward_wheel_force = -forward_wheel_force;
  if(v_wheel_sideward < 0.)
    sideward_wheel_force = -sideward_wheel_force;

  /* Rotate into local (N-E-D) axes */

  v_F_wheel_local.r[0] = forward_wheel_force *cos_wheel_hdg_angle
                       - sideward_wheel_force*sin_wheel_hdg_angle;
  v_F_wheel_local.r[1] = forward_wheel_force *sin_wheel_hdg_angle
                       + sideward_wheel_force*cos_wheel_hdg_angle;
  v_F_wheel_local.r[2] = reaction_normal_force;

  /* Convert reaction force from local (N-E-D) axes to body (X-Y-Z) */

  tempF = LocalToBody * v_F_wheel_local;

  /* Calculate moments from force and offsets in body axes */

  tempM = v_P_wheel_cg_body * tempF;

}


/**
 * Calculate the sum of forces and moments resulting from
 * the hardpoints interacting with the environment.
 *
 * \param inputs          Pointer to controller input class
 * \param env             Pointer to FDM's environment interface
 * \param LocalToBody     Transformation matrix from local to body coordinates
 * \param v_P_CG_Rwy      Position of CG in runway coordinates
 * \param v_R_omega_body  Angular rates in body coordinates
 * \param v_V_local_rel_ground  Velocity relative to ground in local coordinates
 */
void WheelSystem::update( TSimInputs* inputs,
                          FDMEnviroment* env,
                          CRRCMath::Matrix33 LocalToBody,
                          CRRCMath::Vector3  const& v_P_CG_Rwy,
                          CRRCMath::Vector3  const& v_R_omega_body,
                          CRRCMath::Vector3  const& v_V_local_rel_ground,
                          SCALAR psi)
{
  int i;                        /* per wheel loop counter */
  int num_wheels = wheels.size();

  /*
   * Execution starts here
   */
  
  /*
   * Create a local copy of the current simulation inputs.
   * The wheels hold a pointer to this local copy for their update.
   */
  wheel_inputs = *inputs;

  v_Forces  = CRRCMath::Vector3();  /* Initialize sum of forces... */
  v_Moments = CRRCMath::Vector3();  /* ...and moments  */
      
  for (i=0;i<num_wheels;i++)     /* Loop for each wheel */
  {
    wheels[i].update( env,
                      LocalToBody,
                      v_P_CG_Rwy,
                      v_R_omega_body,
                      v_V_local_rel_ground,
                      psi);

    /* Sum forces and moments across all wheels */
    v_Forces  += wheels[i].tempF;
    v_Moments += wheels[i].tempM;
  }
}
#if 0
  /*
  * Calculate height minimum of the horizontal plane  so that this wheel is above the ground  
  *
  */ 
 double Wheel::WheelHeight(double dPsi, double X, double Y)
  {
  double rPsi = dPsi * M_PI/180;
  CRRCMath::Vector3 v_P_wheel_cg_body = v_P;
  if (hpt != NULL)
  {
    hpt->transform(v_P_wheel_cg_body);
  }
  double x = X + v_P_wheel_cg_body.r[0]*cos(rPsi) + v_P_wheel_cg_body.r[1]*sin(rPsi);
  double y = Y - v_P_wheel_cg_body.r[0]*sin(rPsi) + v_P_wheel_cg_body.r[1]*cos(rPsi);
 
  double z_earth = -1;///////*Global::scenery->getHeight(x,y);
  return z_earth;
  }
  
 /*
  * Calculate height (at CG) minimum of the horizontal plane so that all the wheels are above the ground 
  *
  */ 
 double WheelSystem::WheelsHeight(double dPsi, double X, double Y)
  {
    double height=0;
    int num_wheels = wheels.size();
    for (int i=0;i<num_wheels;i++)     /* Loop for each wheel */
    {
      double h = wheels[i].WheelHeight( dPsi, X, Y);
      if(h>height) height= h;
    }
  return height;
  }

#endif

/**
 * Create a WheelSystem with no hardpoints
 */
WheelSystem::WheelSystem()
: v_Forces(0, 0, 0), v_Moments(0,0,0), span_ft(0), dZLow(0), dZHigh(0) 
{
}

/**
 * Initialize a WheelSystem from an XML file.
 *
 * \param ModelFile   pointer to file class
 * \param def_span    default span if no hardpoints are found to calculate it
 */
void WheelSystem::init(SimpleXMLTransfer *ModelFile, SCALAR  def_span)
{
  Wheel              wheel(this);
  SimpleXMLTransfer  *e, *i;
  unsigned int       uSize;
  double             x, y, z;
  double             to_ft;
  double             to_lbf_per_ft;
  double             to_lbf_s_per_ft;
  double             to_lbf;
  CRRCMath::Vector3  pCG;
   
  /**
   * Tracks wingspan [m]
   */
  double span = 0;
  span_ft = 0.0;
  
  
  //
  pCG = CRRCMath::Vector3(0, 0, 0);
  if (ModelFile->indexOfChild("CG") >= 0)
  {
    i = ModelFile->getChild("CG");
    pCG.r[0] = i->attributeAsDouble("x", 0);
    pCG.r[1] = i->attributeAsDouble("y", 0);
    pCG.r[2] = i->attributeAsDouble("z", 0);
    
    if (i->attributeAsInt("units") == 1)
      pCG *= M_TO_FT;
  }
  
  // let's assume that there is nothing below/above the CG:
  dZLow    = 0;
  dZHigh   = 0;
  
  wheels.clear();
  
  i = ModelFile->getChild("wheels");
  switch (i->getInt("units"))
  {
   case 0:    
    to_ft           = 1;
    to_lbf_per_ft   = 1;
    to_lbf_s_per_ft = 1;
    to_lbf          = 1;
    break;
   case 1:
    to_ft           = M_TO_FT;
    to_lbf_per_ft   = FT_TO_M / LBF_TO_N;
    to_lbf_s_per_ft = FT_TO_M / LBF_TO_N;
    to_lbf          = N_TO_LBF;
    break;
   default:
    {
      throw std::runtime_error("Unknown units in wheels");
    }
    break;
  }        
  uSize = i->getChildCount();
  for (unsigned int n=0; n<uSize; n++)
  {
    // we assign the child number as a unique ID for
    // debugging
    wheel.nID = n;
    
    e = i->getChildAt(n);
    
    x = e->getDouble("pos.x") * to_ft - pCG.r[0];
    y = e->getDouble("pos.y") * to_ft - pCG.r[1];
    z = e->getDouble("pos.z") * to_ft - pCG.r[2];
    wheel.v_P = CRRCMath::Vector3(x, y, z);
    
    // let's see if this wheel is coupled to an animation
    wheel.anim_name = e->getString("pos.animation", "");
    if (wheel.anim_name != "")
    {
      std::cout << "WheelSystem::init: hardpoint is coupled to anim ";
      std::cout << wheel.anim_name << std::endl;
    }
    wheel.hpt = NULL;
    if (ModelFile->indexOfChild("animations") >= 0)
    {
      SimpleXMLTransfer *a = ModelFile->getChild("animations");
      unsigned int numAnims = a->getChildCount();
      for (unsigned int animIndex = 0; animIndex < numAnims; animIndex++)
      {
        SimpleXMLTransfer *an = a->getChildAt(animIndex);
        if (an->getString("object.name", "") == wheel.anim_name)
        {
          printf("Found %s animation for wheel %s\n",
                  an->getString("type", "<unknown>").c_str(),
                  wheel.anim_name.c_str());
          wheel.hpt = new HardPointRotation(an, wheel_inputs);
          break;
        }
      }
    }

    wheel.spring_constant    = e->getDouble("spring.constant") * to_lbf_per_ft;
    wheel.spring_damping     = e->getDouble("spring.damping")  * to_lbf_s_per_ft;
    wheel.max_force          = e->getDouble("spring.max_force", 9999) * to_lbf;
    wheel.percent_brake      = e->getDouble("percent_brake");
    wheel.caster_angle_rad   = e->getDouble("caster_angle_rad");

    if (e->indexOfChild("steering") >= 0)
    {
      std::string s = e->getString("steering.mapping", "NOTHING");
      wheel.steering_max_angle = e->getDouble("steering.max_angle", 1.0);
      wheel.steering_mapping   = XMLModelFile::GetSteering(s);
    }
    else
    {
      wheel.steering_mapping   = TSimInputs::smNOTHING;
      wheel.steering_max_angle = 0;
    }
    wheels.push_back(wheel);
    
    // track wingspan
    if (span < y)
      span = y;
    // lowest point?
    if (dZLow < z)
      dZLow = z;
    // highest point?
    if (dZHigh > z)
      dZHigh = z;
  }
  span_ft  = 2 * span;

  // just in case: if there were no hardpoints, use the reference span
  if (span_ft == 0.0)
  {
    span_ft = def_span;
  }
}

