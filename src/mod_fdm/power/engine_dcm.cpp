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
#include "engine_dcm.h"

#include <iostream>
#include "../../mod_misc/filesystools.h"
#include "../../mod_misc/lib_conversions.h"
#include "../../mod_math/linearreg.h"

// Wirkungsgrad des Drehzahlstellers
#define  ETA_STELLER  0.95

#define  THR_P_S 3.0

inline double sqr(double val) { return(val*val); };


Power::Engine_DCM::Engine_DCM() : Gearing()
{
  k_r = 0;
  J   = 0;
  throttle.init(0, THR_P_S);
}

void Power::Engine_DCM::ReloadParams(SimpleXMLTransfer* xml)
{
  Gearing::ReloadParams(xml);
  
  SimpleXMLTransfer* eng;
  bool               fExtern = true;
  
  if (xml->indexOfAttribute("filename") >= 0)  
    eng = new SimpleXMLTransfer(FileSysTools::getDataPath("models/engine/" + xml->getString("filename") + ".xml", true));
  else
  {
    eng     = xml;
    fExtern = false;
  }

  if (eng->attributeAsInt("calc", 0) != 0)
  {
    SimpleXMLTransfer* sp;
    SimpleXMLTransfer* i;
    unsigned int       size;
    double             U_K;
    double             I_M;
    double             omega;
    T_LinearReg        lr;
    
    lr.init();
    sp   = eng->getChild("data");
    size = sp->getChildCount();
    for (unsigned int uCnt=0; uCnt<size; uCnt++)
    {
      i     = sp->getChildAt(uCnt);
      U_K   = i->getDouble("U_K");
      I_M   = i->getDouble("I_M");
      omega = i->getDouble("n") * 2 * M_PI;
      
      lr.add(omega/I_M, U_K/I_M);
    }
    lr.calc();
    R_I = lr.get_a();
    k_M = lr.get_b();
        
    sp   = eng->getChild("data_idle");
    size = sp->getChildCount();
    if (size == 1)
    {
      M_r = sp->getDouble("data.I_M") * k_M;
      k_r = 0;
    }
    else
    {
      lr.init();
      for (unsigned int uCnt=0; uCnt<size; uCnt++)
      {
        i     = sp->getChildAt(uCnt);
        U_K   = i->getDouble("U_K");
        I_M   = i->getDouble("I_M");
        omega = (U_K - R_I * I_M)/k_M;
        lr.add(omega, I_M);
      }
      lr.calc();
      // I_0(omega) = a     + b     * omega
      // M_r(omega) = a*k_M + b*k_M * omega
      M_r = lr.get_a() * k_M;
      k_r = lr.get_b() * k_M;
    }
  }
  else
  {
    R_I = eng->getDouble("R_I");
    k_M = eng->getDouble("k_M");
    // Im Leerlauf verursacht das Reibmoment einen Leerlaufstrom.
    M_r = eng->getDouble("I_0") * k_M;
    k_r = 0;
  }
  J = eng->getDouble("J_M");
  throttle_rate_max = eng->getDouble("rate_max", THR_P_S);
  nLog = eng->getInt("log", 0);

  if (fExtern)
    delete eng;

  std::cout << "Engine_DCM : R_I=" << R_I << " Ohm, M_r=" << M_r << " Nm, k_r="
    << k_r << ", k_M=" << k_M << " Nm/A, J_M=" << J << " kg m^2, rate_max="
    << throttle_rate_max << "\n";
}

void Power::Engine_DCM::step(PowerValuesStep* values)
{
  double M_M;
  double omega = i*values->omega;

  // battery empty?
  if (values->U < 0.01)
  {
    throttle.init(0, throttle_rate_max);
  }
  else
  {
    // limit throttle change and value
    double in = values->inputs->throttle;
    if (in > 1)
      in = 1;
    throttle.step(values->dt, in);
  }
  
  // todo:
  // In case of throttle being zero, an electric engine will not apply any torque.
  // The controller doesn't force voltage to zero, but uses freewheeling. It would
  // do so with a brake enabled, but in this system, the brake is modeled at the 
  // shaft. So here, voltage=0 should just be a free running motor. However, this 
  // is not that easy in any case and maybe a change like this would mean worse
  // realism regarding combustion engines?
  // 
  // voltage applied to motor
  double U_K = throttle.val * values->U * ETA_STELLER;

  // Generatorspannung
  double U_Gen = omega * k_M;

  //  motor current
  double I_M = (U_K - U_Gen) / R_I;

  // Aeusseres Moment
  M_M = k_M * I_M - k_r * omega;
  
  // Das Reibmoment wirkt immer der aktuellen Drehzahl entgegen
  if (omega > 0)
    M_M -= M_r;
  else
    M_M += M_r;

  values->moment_shaft += M_M*i;
  
  // Drain battery. But it can't be charged. Note that this is PWM, so the 
  // mean battery current is t_on/t_cycle*I_M instead of I_M.
  if (I_M > 0)
    values->I += I_M * throttle.val;
  
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
          std::cout << "Engine_dcm:";
          std::cout.width(11);
          std::cout << throttle.val << "      ,";
          std::cout.width(11);
          std::cout << values->U << " V    ,";
          std::cout.width(11);
          std::cout << I_M << " A    ,";
          std::cout.width(11);
          std::cout << (omega*60/(2*M_PI)) << " rpm  ,";
          std::cout << "\n";
        }
      }
  }  
}

void Power::Engine_DCM::ReloadParams_automagic(SimpleXMLTransfer* xml)
{
  SimpleXMLTransfer* e = xml->getChild("battery.shaft.engine");

  throttle_rate_max = e->getDouble("rate_max", THR_P_S);
    
  // A gearing is needed anyway:
  e->setAttributeOverwrite("gearing.J",   "0");

  // Read rotational speed and torque (has just been written by propeller)
  double M_P = xml->getDouble("battery.shaft.propeller.automagic.M_P");
  double n_P = xml->getDouble("battery.shaft.propeller.automagic.n_P");

  // Mit dem Wirkungsgrad des Getriebes ergibt sich die aufzubringende 
  // mechanische Leistung
  double P_mech = M_P * 2 * M_PI * n_P;  

  // Use at least 10V, but 24V at 1kW.
  double U   = 10; // V
  if (P_mech > 60)
    U = U + P_mech * (24-U)/(1000-60);
  // Set voltage I need from battery
  e->setAttribute("automagic.U", doubleToString(U/ETA_STELLER));

  // Position der Polstelle
  double omega_p = e->getDouble("automagic.omega_p");
  k_M = U/omega_p;
  e->setAttribute("k_M", doubleToString(k_M));

  // maximaler Wirkungsgrad bei dieser Spannung
  double eta_opt = e->getDouble("automagic.eta_opt");
  double R_I_mul_I_0 = U * (eta_opt - 2*sqrt(eta_opt) + 1);

  // Wirkungsgrad im geforderten Betriebspunkt
  double eta_nenn = e->getDouble("automagic.eta");
  // Winkelgeschwindigkeit, bei welcher dieser Wirkungsgrad erreicht
  // wird (und zwar vom Maximum aus in Richtung kleinerer Drehzahl).    
  double omega_nenn = (-sqrt(sqr(eta_nenn-1)*U*U-2*(eta_nenn+1)*R_I_mul_I_0*U + sqr(R_I_mul_I_0))
                       +(eta_nenn+1)*U - R_I_mul_I_0)
                      /(2*k_M);

  // Jetzt muss bei dieser Drehzahl noch die gewuenschte mech. Leistung
  // erreicht werden:
  double I_M = P_mech / (U * eta_nenn);
  // Set current I need from battery. Note that this is PWM, so the 
  // mean battery current is t_on/t_cycle*I_M instead of I_M. However, in this
  // case a PWM ratio of 1 is assumed.
  e->setAttribute("automagic.I", doubleToString(I_M));

  R_I = (U - k_M * omega_nenn) / I_M;
  M_r = (R_I_mul_I_0 / R_I) * k_M;
  k_r = 0;
  e->setAttribute("R_I", doubleToString(R_I));
  e->setAttribute("automagic.M_r", doubleToString(M_r));
  e->setAttribute("I_0", doubleToString(M_r / k_M));

  // calc gearing
  i   = omega_nenn/2.0/M_PI/n_P;
  e->setAttribute("gearing.i", doubleToString(i));

  // Inertia
  //  P = M * omega
  //  M = P/omega
  J = P_mech/omega_nenn * 0.5/omega_nenn;
  e->setAttribute("J_M", doubleToString(J));
  
  nLog = e->getInt("log", 0);
}

void Power::Engine_DCM::InitStates(CRRCMath::Vector3 vInitialVelocity, double& dOmega)
{
  throttle.init(0, throttle_rate_max);
}

