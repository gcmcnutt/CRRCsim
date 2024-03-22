/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2008-2009 Jens Wilhelm Wulf (original author)
 * Copyright (C) 2008 Jan Reucker
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
  

#include "crrc_fdm.h"

#include "global.h"
#include "mod_landscape/crrc_scenery.h"
#include "mod_windfield/windfield.h"
#include "mod_env/earth/atmos_62.h"
#include "mod_env/earth/ls_gravity.h"
#include "mod_fdm/fdm.h"

CRRC_FDM_Env::CRRC_FDM_Env(SimpleXMLTransfer* cfg)
{
  // instantiate list of controllers from global config file,
  // so these controllers are used no matter which model is loaded
  controllers.clear();  
  int idx = cfg->indexOfChild("controllers");
  if (idx >= 0)
    Controller::LoadList(cfg->getChildAt(idx), controllers);
}

float CRRC_FDM_Env::GetSceneryHeight(float x_north, float y_east)
{
  return(Global::scenery->getHeight(x_north, y_east));
}

int CRRC_FDM_Env::CalculateWind(double  X_cg,      double  Y_cg,     double  Z_cg,
                                double& Vel_north, double& Vel_east, double& Vel_down)
{
  return(calculate_wind(X_cg,      Y_cg,     Z_cg,
                        Vel_north, Vel_east, Vel_down));
}

int CRRC_FDM_Env::CalculateWindGrad(double X_cg, double Y_cg, double Z_cg, double delta_space,
                                    CRRCMath::Matrix33& m_V_grad)
{
  return(calculate_wind_grad(X_cg, Y_cg, Z_cg, delta_space, m_V_grad));
}

void CRRC_FDM_Env::InitializeWindGust()
{
  initialize_gust();
}

void CRRC_FDM_Env::CalculateWindGust(double dt, double altitude, double V_rel_wind, double b,
                                     CRRCMath::Vector3 v_V_local_airmass,
                                     CRRCMath::Matrix33 LocalToBody,
                                     CRRCMath::Vector3& v_V_gust_body,
                                     CRRCMath::Vector3& v_R_omega_gust_body)
{
  calculate_gust(dt, altitude, V_rel_wind, b, v_V_local_airmass, LocalToBody,
                 v_V_gust_body, v_R_omega_gust_body);
}

double CRRC_FDM_Env::GetRho(double altitude)
{
  double origin_altitude = Global::scenery->getOriginAltitude();
  return(ls_atmos_rho(altitude + origin_altitude));
}

double CRRC_FDM_Env::GetG(double altitude)
{
  return(ls_gravity_g(altitude));
}

void CRRC_FDM_Env::ControllerCallback(double      dt, 
                                      FDMBase*    fdm,
                                      TSimInputs* pInputsFromUser,
                                      TSimInputs* pInputsToFDM)
{
  pInputsToFDM->CopyFrom(pInputsFromUser);

  // Process controllers
  for (unsigned int n=0; n<controllers.size(); n++)
    controllers[n]->Calc(dt, fdm, pInputsFromUser, pInputsToFDM);
}

void CRRC_FDM_Env::ResetControllers()
{
  for (unsigned int n=0; n<controllers.size(); n++)
    controllers[n]->Reset();
}

CRRC_FDM_Env::~CRRC_FDM_Env()
{
  for (unsigned int n=0; n<controllers.size(); n++)
    delete controllers[n];
}

void CRRC_FDM_Env::AddLogMsg(std::string message)
{
  LOG(message);
}
