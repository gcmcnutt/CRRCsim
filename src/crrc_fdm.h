/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2008-2009 Jens Wilhelm Wulf (original author)
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
  

#ifndef CRRC_FDM_ENV_H
# define CRRC_FDM_ENV_H

#include "mod_math/matrix33.h"
#include "mod_misc/SimpleXMLTransfer.h"
#include "mod_fdm/fdm_env.h"
#include "mod_cntrl/controller.h"

/**
 * Connects CRRCSim to the module "FDM"
 * 
 * @author Jens Wilhelm Wulf
 */
class CRRC_FDM_Env : public FDMEnviroment
{
public:

  CRRC_FDM_Env(SimpleXMLTransfer* cfg);
  virtual ~CRRC_FDM_Env();
  
  /**
   *  Get the height at a distinct point.
   *  \param x x coordinate (positive north)
   *  \param y y coordinate (positive east)
   *  \return terrain height at this point in ft
   */
  virtual float GetSceneryHeight(float x_north, float y_east);
  
  /**
   * Calculate the wind velocities in all three axes in the given position.
   * Returns 1 if this position is outside of the grid.
   * X/Y/Z -- north/east/down
   */
  virtual int CalculateWind(double  X_cg,      double  Y_cg,     double  Z_cg,
                            double& Vel_north, double& Vel_east, double& Vel_down);
                            
  /**
   * Calculate the gradient of wind velocity in the north/east/down frame.
   */
  virtual int CalculateWindGrad(double X_cg, double Y_cg, double Z_cg, double ref_len,
                                CRRCMath::Matrix33& m_V_grad);
                                
  /**
   * Initialize gust (a.k.a wind turbulence) linear and rotational
   * velocities in body axes.
   */
  virtual void InitializeWindGust();

  /**
   * Given the time since last iteration updates gust linear
   * and rotational velocities in body axes.
   */
  virtual void CalculateWindGust(double dt, double altitude, double V_rel_wind, double b,
                                 CRRCMath::Vector3 v_V_local_airmass,
                                 CRRCMath::Matrix33 LocalToBody,
                                 CRRCMath::Vector3& v_V_gust_body,
                                 CRRCMath::Vector3& v_R_omega_gust_body);

  /**
   * Returns gravitational acceleration at height 'altitude'
   */
  virtual double GetG(double altitude);

  /**
   * Returns air density at height 'altitude'
   */
  virtual double GetRho(double altitude);
  
  /**
   * This can be used to integrate one or many controllers into the simulation loop.
   * If you don't want to do this, simply copy the contents of pInputsFromUser to pInputsToFDM.
   * There might be FDMs which do not use this callback, because their author thought
   * using them with a control loop is not of much use.
   */
  virtual void ControllerCallback(double dt, FDMBase* fdm, TSimInputs* pInputsFromUser, TSimInputs* pInputsToFDM);

  void ResetControllers();
  
  virtual void AddLogMsg(std::string message);
  
private:
  
  /**
   * List of active controllers
   */
  std::vector<Controller*> controllers;
};

#endif
