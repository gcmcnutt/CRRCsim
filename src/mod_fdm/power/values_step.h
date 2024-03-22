/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2008 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2008 - Jan Reucker
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
#ifndef VALUES_STEP_H
# define VALUES_STEP_H

# include "../../mod_math/vector3.h"
# include "../fdm_inputs.h"


namespace Power
{

  /**
   * This class is part of the power system. To simply use the system, you should not
   * access or call any of its members. Please take a look at Power instead.
   *
   * This structure is used to keep low the number of parameters to pass in the power system.
   *
   * @author Jens Wilhelm Wulf
   */
  struct PowerValuesStep
  {
    /**
     * timestep [s]
     */
    double      dt;

    /**
     * pointer to control inputs
     */
    TSimInputs* inputs;

    /**
     * Sums up force created by power system [N] (body axes).
     */
    CRRCMath::Vector3*    force;

    /**
     * Sums up torque created by power system [Nm] (body axes).
     */
    CRRCMath::Vector3*    moment;

    /**
     * Battery voltage [V]
     */
    double      U;
    
    /**
     * Sums up current flowing from the battery [A].
     */
    double      I;
    
    /**
     * Speed of shaft [rad/s].
     */
    double      omega;
    
    /**
     * Sums up torque applied to shaft [Nm].
     */
    double      moment_shaft;
    
    /**
     * Rotational sped of the last propeller calculated [1/s].
     */
    double      dPropFreq;
    
    /**
     * Velocity of power system relative to airmass, [m/s].
     */
    CRRCMath::Vector3 VRelAir;
    
     /**
      * Lowest relative battery capacity left (0..1).
      */
     double  dBatCapLeftMin;
  };
}

#endif
