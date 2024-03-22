/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2008 - Jens Wilhelm Wulf (original author)
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
#ifndef CONTROLLER_H
# define CONTROLLER_H

#include "../mod_misc/SimpleXMLTransfer.h"
#include "../mod_fdm/fdm.h"
#include "../mod_fdm/fdm_env.h"
#include "../mod_fdm/fdm_inputs.h"

/**
 * Base class/interface for a controller (autopilot or whatever).
 * Does also include static methods which might be needed by a controller or
 * by the code which wants to use controllers.
 * 
 * @author Jens W. Wulf
 */
class Controller
{
public:
  
  /**
   * This one has to be implemented by every controller.
   * 
   * dt               is the timestep since the previous call
   * fdm              points to the fdm in use
   * pInputsFromUser  contains user (stick) inputs
   * pInputsToFDM     is the data the controller might set or change
   */
  virtual void Calc(double      dt, 
                    FDMBase*    fdm,
                    TSimInputs* pInputsFromUser,
                    TSimInputs* pInputsToFDM) = 0;

  virtual ~Controller() {};

  virtual void Reset() {};
  
  /**
   * Creates a list of controllers according to the xml description in cfg.
   */
  static void LoadList(SimpleXMLTransfer*       cfg,
                       std::vector<Controller*> &controllers);
  
  /**
   * Limits flVal to -0.5 <= flVal <= 0.5
   * Returns 1 if limit to 0.5, returns -1 if limit to -0.5, 0 otherwise.
   */
  static int Limit(float &flVal);
};

#endif
