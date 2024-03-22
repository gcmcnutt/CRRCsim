/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2008 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2007 - Jan Reucker
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
#ifndef SIMPLETHRUST_H
# define SIMPLETHRUST_H

# include "gearing.h"
# include "values_step.h"

namespace Power
{

  /**
   * This class is part of the power system. To simply use the system, you should not
   * access or call any of its members. Please take a look at Power instead.
   *
   * This is a very simple propeller to model CRRCSims first
   * engine: thrust, no matter at which airspeed.
   * 
   * This simple propeller is connected to a shaft via some gearing. 
   * The gearing is described by
   *   - \c i \n
   *     gear ratio
   *   
   * Given \c omega is the speed of the shaft this simple propeller is connected to,
   * it will rotate at <tt>(omega_p = i * omega)</tt> and provide a thrust of <tt>(F = k_F * omega_p)</tt>.
   * 
   * The torque applied to the shaft is <tt>(M = -1 * k_M * omega_p * i)</tt>.
   * 
   * Example for an xml description for direct connection to shaft:
   * 
   \verbatim
   <simplethrust k_F="0.004" k_M="0.00001" />
   \endverbatim
   * 
   * Example for an xml description, connection to shaft via a gearbox:
   * 
   \verbatim
   <simplethrust k_F="0.005" k_M="0.00002" >
     <gearing i="1.13" J="0" />
   </simplethrust>
   \endverbatim
   * 
   * See Power::Gearing for a description of a gearbox.
   *
   * @author Jens Wilhelm Wulf
   */
  class SimpleThrust : public Gearing
  {
    public:
     
     /**
      * 
      */
     SimpleThrust();

     /**
      * virtual base class should have a virtual dtor
      */
     virtual ~SimpleThrust() {};

     /**
      * Go ahead values->dt seconds in the simulation.
      */
     virtual void step(PowerValuesStep* values);

     /** 
      * Load or reload parameters
      */
     virtual void ReloadParams(SimpleXMLTransfer* xml);
     
    private:

     /**
      * Thrust provided is (k_F * i * Shaft::omega)
      */
     double k_F;

     /**
      * Torque applied to shaft is (-1 * k_M * i * Shaft::omega * i)
      */
     double k_M;
  };
};
#endif
