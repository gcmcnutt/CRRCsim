/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2006, 2008, 2009 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2007, 2008 - Jan Reucker
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
#ifndef GEARING_H
# define GEARING_H

# include "../../mod_misc/SimpleXMLTransfer.h"
# include "values_step.h"

namespace Power
{

  /**
   * This class is part of the power system. To simply use the system, you should not
   * access or call any of its members. Please take a look at Power instead.
   *
   * This class models a gearing which connects engines/propellers to a shaft. Everything
   * connected to a shaft must be a subclass of it.
   * 
   * The xml description of a gearing must be written inside of the element which is connected to 
   * the shaft by the gearbox. It looks like this:
   \verbatim
     <gearing i="1.13" J="0" />
   \endverbatim
   * A gearbox might have an inertia \c J which is not zero. This inertia is the value seen by the shaft.
   * The inertia of the propeller/engine is translated to the shaft automatically.
   * 
   * Given \c omega is the speed of the shaft, <tt>i*omega</tt> is the speed of the device which is connected to the shaft
   * using this gearing.
   *
   * @author Jens Wilhelm Wulf
   */
  class Gearing
  {
    public:

     /**
      * 
      */
     Gearing();
     
     /**
      * Read configuration from xml.
      * @param xml
      */
     virtual void ReloadParams(SimpleXMLTransfer* xml);
    
     /**
      * virtual base class should have a virtual dtor
      */
     virtual ~Gearing() {};

     /**
      * Read parameters for automagic construction from xml.
      */
     virtual void ReloadParams_automagic(SimpleXMLTransfer* xml) {};
        
     /**
      * Returns the inertia of the gearing and the device
      * attached to it as seen by the shaft [kg m^2].
      */
     double getJ();

     /**
      * Go ahead values->dt seconds in the simulation.
      * The device, which is attached to a Shaft by this Gearing, adds its
      * forces/torque/current/... to the ones in PowerValuesStep.
      */
     virtual void step(PowerValuesStep* values) = 0;
    
     /**
      * vInitialVelocity is the initial velocity of the airmass relative to the power
      * system. A 'gearing' can write dOmega to tell the shaft to which it is connected
      * about the rotational speed the shaft should have at this velocity.
      */
     virtual void InitStates(CRRCMath::Vector3 vInitialVelocity, double& dOmega) {};
    
    protected:

     /**
      * Inertia of the device attached to the gearing [Nm s = kg m^2]
      */
     double J;

     /**
      * Gear ratio. The propeller/engine rotates at a rate of i * Shaft::omega.
      */
     double i;
     
    private:

     /**
      * Inertia of the gearing as seen by the shaft [Nm s = kg m^2]
      */
     double J_G;
  };
}
#endif
