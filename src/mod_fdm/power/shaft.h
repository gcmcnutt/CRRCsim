/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2008, 2009 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2006, 2008 - Jan Reucker
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
#ifndef SHAFT_H
# define SHAFT_H

# include <vector>
# include "../../mod_math/intgr.h"
# include "../../mod_misc/SimpleXMLTransfer.h"
# include "values_step.h"
# include "gearing.h"

namespace Power
{

  /**
   * This class is part of the power system. To simply use the system, you should not
   * access or call any of its members. Please take a look at Power instead.
   *
   * This class models a shaft to which engines and propellers (any number of them) are connected.
   *
   * For folding props a speed brake is needed, but it didn't work as expected in Engine_DCM. Therefore
   * the simulation of the brake was ripped out of the engine (where the real model proved
   * to be too weak) and has been put into the shaft.
   * So if the brake is enabled, it will force the shaft to stop rotating as soon as the throttle command is zero.
   * 
   * Example for an xml description:
   * 
   \verbatim
   <shaft J="0.0E-5" brake="1">
    ...engines and propellers...
   </shaft>
   \endverbatim
   * \c J is the inertia of the shaft; if \c brake is not zero, this shaft will stop rotating as soon as 
   * the throttle command is zero. This is needed for folding props.
   *
   * @author Jens Wilhelm Wulf
   */
  class Shaft
  {
    public:
     
     /**
      * only used for automagic construction
      */
     Shaft();

     /**
      * Read configuration from xml and create elements below.
      * @param xml
      */
     Shaft(SimpleXMLTransfer* xml);
       
     /**
      * Deallocate all ressources of this shaft.
      */
     ~Shaft();

     /**
      * Resets battery status, initialize states
      */
     void InitStates(CRRCMath::Vector3 vInitialVelocity);
 
     /** 
      * Load or reload parameters
      */
     void ReloadParams(SimpleXMLTransfer* xml);
        
     /** 
      * Load or reload parameters in case of automagic settings
      */
     void ReloadParams_automagic(SimpleXMLTransfer* xml);
     
     /**
      * Go ahead values->dt seconds in the simulation. Calls Gearing::step() for each connected device.
      *
      * @param values relevant data for a step
      */
     void step(PowerValuesStep* values);
        
    private:

     /**
      * 1/inertia of the shaft and everything attached to it [1/(kg m^2)].
      */
     double J_inv;

     /**
      * Speed of the shaft [rad/s].
      */
     CRRCMath::Integrationsverfahren<double> omega;

     /**
      * Engines/propellers connected to this shaft.
      */
     std::vector<Gearing*> gear;

     /**
      * Is there a brake on this shaft?
      */
     bool fBrake;
  };
}
#endif
