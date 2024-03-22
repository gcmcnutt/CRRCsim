/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2006, 2008, 2009 - Jens Wilhelm Wulf (original author)
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
#include "propeller.h"

#include <iostream>
#include "../../mod_misc/filesystools.h"
#include "../../mod_misc/lib_conversions.h"


#define RHO       1.225 // kg/m^3
#define ETA_PROP  0.65

// prop thrust correction coef from:
// Gabriel Staples, "Propeller Static & Dynamic Thrust Calculation"
// http://electricrcaircraftguy.blogspot.it/2014/04/propeller-static-dynamic-thrust-equation-background.html
#define STAPLES_COEF  3.29546 
#define STAPLES_EXP   1.5

Power::Propeller::Propeller() : Gearing()
{
  omega_fold = 5;
  J          = 0;
  dirThrust  = CRRCMath::Vector3(1, 0, 0);
  mulForce   = CRRCMath::Vector3(1, 0, 0);
  mulMoment  = CRRCMath::Vector3(0, 0, 0);
}

void Power::Propeller::ReloadTuneParams(SimpleXMLTransfer* xml)
{
  int idx = xml->indexOfChild("tune");
  if (idx < 0)
  {
    k_F        = 1;
    k_V        = 1;
    k_P        = 1;
    k_Pfac     = 1;
  }
  else    
  {
    SimpleXMLTransfer* sxttune = xml->getChildAt(idx);
    k_F        = sxttune->getDouble("F_V0_scale", 1);
    k_V        = sxttune->getDouble("V_F0_scale", 1);
    k_P        = sxttune->getDouble("P_scale", 1);
    k_Pfac     = sxttune->getDouble("Pfactor_scale", 1);
  }

  std::cout << "Prop tuning: k_F=" << k_F << ", k_V=" << k_V << ", k_P=" << k_P << ", k_Pfac=" << k_Pfac << "\n";
}

void Power::Propeller::CalcDownthrust(SimpleXMLTransfer* xml)
{
  int idx = xml->indexOfChild("pos");
  if (idx < 0)
  {
    downthrust  = 0.;
    rightthrust = 0.;
    dirThrust   = CRRCMath::Vector3(1, 0, 0);
    mulForce    = CRRCMath::Vector3(1, 0, 0);
    mulMoment   = CRRCMath::Vector3(0, 0, 0);
  }
  else    
  {
    SimpleXMLTransfer* sxtpos = xml->getChildAt(idx);
    downthrust  = M_PI * sxtpos->getDouble("downthrust", 0) / 180.;
    rightthrust = M_PI * sxtpos->getDouble("rightthrust", 0) / 180.;
    
    dirThrust = CRRCMath::Vector3(cos(downthrust)*cos(rightthrust), 
                                  cos(downthrust)*sin(rightthrust),
                                  sin(downthrust));
    CRRCMath::Vector3 pos = CRRCMath::Vector3(sxtpos->getDouble("x", 0),
                                              sxtpos->getDouble("y", 0),
                                              sxtpos->getDouble("z", 0));      
    
    // Evaluate rolling, pitching and yawing moment produced by thrust
    // due to distance from CG    
    mulForce  = dirThrust;
    mulMoment = CRRCMath::Vector3(  dirThrust.r[2]*pos.r[1] - dirThrust.r[1]*pos.r[2],
                                  - dirThrust.r[2]*pos.r[0] + dirThrust.r[0]*pos.r[2], 
                                    dirThrust.r[1]*pos.r[0] - dirThrust.r[0]*pos.r[1]);    
  }

  std::cout << "Prop pos   : ";
  mulForce.print("mulForce=", ", ");
  mulMoment.print("mulMoment=", "\n");
}

void Power::Propeller::ReloadParams(SimpleXMLTransfer* xml)
{
  Gearing::ReloadParams(xml);
  
  SimpleXMLTransfer* prop;
  bool               fExtern = true;
  
  if (xml->indexOfAttribute("filename") >= 0)
    prop = new SimpleXMLTransfer(FileSysTools::getDataPath("models/propeller/" + xml->getString("filename") + ".xml", true));
  else
  {
    prop    = xml;
    fExtern = false;
  }
      
  // Note: fdm_heli01 & fdm_mcopter01 only use:
  // a) x component of prop force (force.r[0]) i.e. axial traction
  // b) x component of prop moment (moment.r[0]) i.e. torque moment
  // Thus propeller's "pos" parameters (x,y,downthrust,rightthrust)
  // shall not be used.
  // Additionally for multicopter the propeller's "rotation" parameter 
  // shall not be used since specific parameters for each prop exist.

  D          = prop->getDouble("D");
  H          = prop->getDouble("H");
  J          = prop->getDouble("J");
  rot        = prop->getDouble("rotation", 1);
  omega_fold = prop->attributeAsDouble("n_fold", -1)*2*M_PI;
  nLog       = prop->getInt("log", 0);

  if (fExtern)
    delete prop;    
  
  std::cout << "Propeller  : D=" << D << " m, H=" << H << " m, J=" << J << " kg m^2\n";

  // The scaling factor for thrust/speed/power/P-actor are always read from model file
  // since they likely shall be tuned for each model
  ReloadTuneParams(xml);
  
  // k_F is further multipled by Staples static thrust fit factor
  // NB: Staples's correction factor seems to largely underestimate 
  //     propeller thrust, so it has been disabled for now.
  //k_F *= pow(D/(H * STAPLES_COEF),STAPLES_EXP);
    
  // Der Sturz wird in jedem Fall aus der Modelldatei gelesen, ansonsten muss man ja eine 
  // Propellerdatei fuer jeden Sturz extra haben.
  CalcDownthrust(xml);
}

void Power::Propeller::step(PowerValuesStep* values)
{
  double omega = i*values->omega;
  double n     = omega/(2*M_PI);
    
  if (omega < omega_fold && omega_fold > 0)
  {
    fFolded           = true;
    values->dPropFreq = 0;
  }
  else
  {
    fFolded           = false;
    values->dPropFreq = n;
    
    if (n > 0)
    {
      double V_p = values->inputs->pitch * H * n;
      // small down/right thrust angle approximation applied
      double V_X = values->VRelAir.r[0];
      double V_w = sqrt(values->VRelAir.r[1]*values->VRelAir.r[1]
                      + values->VRelAir.r[2]*values->VRelAir.r[2]);
      double V_y = values->VRelAir.r[1] - rightthrust*V_X;
      double V_z = values->VRelAir.r[2] - downthrust*V_X;
      CRRCMath::Vector3 mulPfac(0, V_y, V_z);

      // thrust force
      filter.step(values->dt, V_p - V_X/(1. + (k_V - 1.)*V_X/V_p));
      double F_X = M_PI * 0.25 * D*D * RHO * k_F * fabs(V_X + 0.5*filter.val) * filter.val;    
    
      // shaft power & torque
      double P = k_P * F_X * (V_X + 0.5*filter.val) / ETA_PROP;
      double M = P/omega * i;

      // P-factor, i.e. yaw & pitching moment due to non-axial inflow
      double M_Pfac = 0;
      
      if (F_X > 0)
      {
        // Effective Translational Lift, see 
        //   http://user.cs.tu-berlin.de/~calle/marvin/dissertation/aerodynamik.html
        // This lift is 'for free', is does not mean more P or M!
        // It is important to model this effect for helicopters, it is unimportant for
        // fixed wing planes (but does no harm in this case).
        double x       = fabs(V_w/V_p);
        const double c = -0.20037;   
        const double d = 0.0825119;  
        const double e = -0.00997873;
        
        if (isfinite(x))
        {
          // I don't know about x>3. Maybe it will never happen, but limiting is save:
          if (x>3)
            x=3;
          
          double fact = 1+c*(x*x)+d*(x*x*x)+e*(x*x*x*x);
          F_X = F_X / fact;
        }
        
        // P-factor, i.e. yaw & pitching moment due to non-axial inflow
        M_Pfac = - k_Pfac * M_PI * 0.25 * D*D * 0.5 * V_p*V_p / omega;
      }
       
      values->moment_shaft -= M;

      *values->force += mulForce * F_X;
      *values->moment += mulMoment * F_X - dirThrust * (rot * M) + mulPfac * (rot * M_Pfac);
    
      switch (nLog)
      {
        case 1:
          {
            static int logcnt = 0;
            if (logcnt++ >= 10)
            {
              logcnt = 0;
              
              std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
              std::cout.precision(3);
              std::cout << "Propeller :";
              std::cout.width(11);
              std::cout << V_X << " m/s  ,";
              std::cout.width(11);
              std::cout << F_X << " N    ,";
              std::cout.width(11);
              std::cout << n*60 << " rpm  ,";
              std::cout.width(11);
              std::cout << P << " W    ,";
              std::cout << "\n";
            }
          }
      }  
    }
  }
}

void Power::Propeller::ReloadParams_automagic(SimpleXMLTransfer* xml)
{
  SimpleXMLTransfer* prop = xml->getChild("battery.shaft.propeller");

  D          = prop->getDouble("D");
  H          = prop->getDouble("H");
  J          = prop->getDouble("J");
  rot        = prop->getDouble("rotation", 1);
  omega_fold = prop->attributeAsDouble("n_fold", -1)*2*M_PI;
  nLog       = prop->getInt("log", 0);
  
  std::cout << "Propeller  : D=" << D << " m, H=" << H << " m, J=" << J << " kg m^2\n";
    
  // The scaling factors for thrust/speed/power/P-actor are always read from model file
  // since they likely shall be tuned for each model
  ReloadTuneParams(prop);
  k_F        = 1.;
  k_V        = 1.;
  k_P        = 1.;
  
  double F = xml->getDouble("F");
  double V = xml->getDouble("V"); 
  
  // Calculate rotational speed and torque needed:
  //  F = M_PI * 0.25 * D*D * RHO * (V_X + filter.val/2) * filter.val;
  //  F = M_PI * 0.25 * D*D * RHO * (V + (Hn-V)/2) * (Hn-V);
  //  F = M_PI * 0.25 * D*D * RHO * (V/2 + Hn/2) * (Hn-V);
  double n = sqrt( (8*F/(M_PI*D*D*RHO)) + V*V)/H;    
  double M = F * (V + (V + H*n)/2) / (2*M_PI*n) * i / ETA_PROP;
  
  // Save these values so the engine can adjust itself to them:
  prop->setAttribute("automagic.n_P", doubleToString(n));
  prop->setAttribute("automagic.M_P", doubleToString(M));

  // Der Sturz wird in jedem Fall aus der Modelldatei gelesen, ansonsten muss man ja eine 
  // Propellerdatei fuer jeden Sturz extra haben.
  CalcDownthrust(prop);
}

void Power::Propeller::InitStates(CRRCMath::Vector3 vInitialVelocity, double& dOmega)
{
  filter.init(0, 0);
  fFolded = true;
  if (omega_fold < 0)
    dOmega = 2 * M_PI * vInitialVelocity.r[0] / H;
}
