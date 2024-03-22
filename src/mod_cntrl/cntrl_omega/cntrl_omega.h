/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2009 - Jens Wilhelm Wulf (original author)
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
#ifndef CNTRL_OMEGA_H
# define CNTRL_OMEGA_H

#include "../controller.h"
#include "../../mod_math/pt1.h"

#include <fstream>

/**
 * This PID-controller makes stick inputs control rotation rates.
 * 
 * @author Jens W. Wulf
 */
class Cntrl_Omega : public Controller
{
public:
  
  Cntrl_Omega(SimpleXMLTransfer* cfg);
  
  virtual void Reset();
  
  virtual void Calc(double      dt, 
                    FDMBase*    fdm,
                    TSimInputs* pInputsFromUser,
                    TSimInputs* pInputsToFDM);

  virtual ~Cntrl_Omega();
      
private:

  double scale_a;
  double scale_b;
  double scale_exp;

  /**
   * Scale roll and nick with massive expo for loops and flips
   */
  double Scale(double in);
  
  class Controller
  {
  private:
    double scale;
    double kp;
    double ki;
    double kd;
  public:
    double val_old;
    CRRCMath::PT1 filter;
    double integrator;
    
    void ReadData(SimpleXMLTransfer* cfg)
    {
      scale = cfg->getDouble("scale");
      kp    = cfg->getDouble("kp");
      ki    = cfg->getDouble("ki");
      kd    = cfg->getDouble("kd", 0);
      filter.SetTau(cfg->getDouble("tau_d", 0));
      val_old = 0;
    }
    
    double Step(double dt, double setpoint, double val)
    {
      double diff = setpoint*scale - val;
      double out;
      
      filter.step(dt, (val-val_old)/dt);
      val_old = val;
      
      double pd = diff * kp + kd*filter.val;
      
      if (pd > 1)
        pd = 1;
      else if (pd < -1)
        pd = -1;
      else
        integrator += diff * dt;
      
      if (integrator * ki > 1)
        integrator = 1 / ki;
      else if (integrator * ki < -1)
        integrator = -1 / ki;
      
      out = integrator * ki + pd;
      return(out);
    }
  };
  Controller controllers[3];
  
  bool   fLog;
  
  std::ofstream outlog;
};

#endif
