/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2006, 2008, 2009 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2006, 2007, 2010 - Jan Reucker
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
#ifndef ENGINE_DCM_H
# define ENGINE_DCM_H

# include "gearing.h"
# include "values_step.h"
# include "../../mod_math/ratelim.h"

namespace Power
{

  /**
   * This class is part of the power system. To simply use the system, you should not
   * access or call any of its members. Please take a look at Power instead.
   *
   * This class models a direct current motor. The model is quite realistic. However, modeling
   * the emc brake proved to be not strong enough to stop a Propeller in flight. But this is a
   * problem of the Propeller I think. The brake is modelled by Shaft now.
   * 
   * The xml configuration of an engine looks like this (without gearbox to shaft):
   \verbatim
   <engine k_M="4.3e-03" R_I="0.185" J_M="1.79e-05" I_0="1.01" />
   \endverbatim
   * 
   * 
   * Example for an xml description, connection to shaft via a gearbox:
   *
   \verbatim
   <engine k_M="4.3e-03" R_I="0.185" J_M="1.79e-05" I_0="1.01" >
     <gearing i="1.13" J="0" />
   </engine>
   \endverbatim
   * The inertia \c J of the engine is translated to the shaft automatically. The inertia \c J of the gearing is
   * the value seen by the shaft.
   *
   * See Power::Gearing for a description of a gearbox.
   *
   * It is possible to read the parameters of a engine from a separate file. In this case use
   * something like
   \verbatim
   <engine filename="astro_cobalt"/>
   \endverbatim
   \verbatim
   <engine filename="astro_cobalt">
     <gearing i="1.13" J="0" />
   </engine>
   \endverbatim
   * instead of writing down the parameters directly. The system will try to load a file 
   * <tt>./models/engine/astro_cobalt.xml</tt> which might look like this:
   \verbatim
   <?xml version="1.0"?>
   <!--
    Astro Cobalt 05 at 10 V
   
    Data taken from
    Retzbach, Ludwig: Ratgeber Elektroflug, Neckar Verlag, 1991.
  
    M_r = 1.15E-2; // Nm
    I_0 = M_r/k_M = 2.74A
  
    J_M is just guessed.
  
    -->

   <engine R_I="0.08" k_M="0.42E-2" I_0="2.74" J_M="1.6E-6" >
   </engine>
   \endverbatim
   *
   * Given \c U_K is the voltage applied to the engine and omega is its speed,
   * it will draw a current of
   * <tt>I_M = (U_K - omega * k_M) / R_I</tt>
   * and will apply a torque to the gearing which is
   * <tt>M_M = k_M * (I_M - I_0)</tt>.
   * 
   *
   * Finding a complete parameter set for a specific engine can be impossible,
   * but luckily the parameters can be calculated from measured data.
   * Given the current draw for the idle engine and
   * voltage, current draw and speed
   * values for at least two different load points (one of them may be the 
   * idle point as well), all electric parameters can be calculated. 
   
  
   * A worked example: Speed 400 with flux ring
   *
   * For one model I wanted to simulate a Speed 400 (aka Mabuchi RS-380 PH)
   * with an additional flux ring.
   * I found an Excel sheet on the Internet that contained motor data for
   * exactly this engine:
   *
   \verbatim
    U_K [V]  I_M [A]   n [rpm]  n [1/s]   remark
    -----------------------------------------------------------------------------------
     7.96     0.94     22290    371.5     idle, n = n_0, U_K = U_0 and I_M = I_0
     7.37     7.47     13740    229       near max. load
   \endverbatim
   * J_M, the engine's rotor's inertia, can be found in the manufacturer's
   * data sheet, or it has to be guessed. I assumed 1.0E-6 for the
   * Speed 400. You can estimate it by regarding the rotor as a
   * solid iron cylinder of mass m (in kg) and diameter d (in m)
   * using the formula
   *
   \verbatim
   J_M = 0.5 * m * d^2 / 4
   \endverbatim
   *
   * All this resulted in the following engine file:
   *
   \verbatim
   <?xml version="1.0"?>
   <!--
     Mabuchi 380 (aka Speed 400) with additional flux ring
     
     J_M is just guessed.
   -->
   
    <engine_dcm  J_M="1.0E-6" calc="1">
      <data>
        <data U_K="7.96" I_M="0.94" n="371.5" />
        <data U_K="7.37" I_M="7.47" n="229.0" />
      </data>  
      <data_idle>
        <data I_M="0.94" />
      </data_idle>
    </engine_dcm>
   \endverbatim
   * 
   * There must be at least two entries with different load points, but 
   * if you have more, just supply all of them. 
   *
   * If you only have one set of idle data, the voltage
   * does not matter.
   * But if you can provide several values for idle current at a certain voltage, do so. Replace
   \verbatim
      <data_idle>
        <data I_M="0.94" />
      </data_idle>
   \endverbatim
   * by something like
   \verbatim
      <data_idle>
        <data U_K="7.96" I_M="0.94" />
        <data U_K="6.13" I_M="0.87" />
        <data U_K="5.07" I_M="0.82" />
      </data_idle>
   \endverbatim
   * 
   * @author Jens Wilhelm Wulf
   */
  class Engine_DCM : public Gearing
  {
    public:

     /**
      * 
      */
     Engine_DCM();
    
     /**
      * virtual base class should have a virtual dtor
      */
     virtual ~Engine_DCM() {};
           
     /** 
      * Load or reload parameters
      */
     void ReloadParams(SimpleXMLTransfer* xml);
        
     /** 
      * Load or reload parameters in case of automagic settings
      */
     void ReloadParams_automagic(SimpleXMLTransfer* xml);

     /**
      * Go ahead values->dt seconds in the simulation.
      */
     virtual void step(PowerValuesStep* values);
    
     virtual void InitStates(CRRCMath::Vector3 vInitialVelocity, double& dOmega);

    private:

     /**
      * resistance [Ohm]
      */
     double R_I;
     
     /**
      * Friction [Nm]. This part does not depend on rotational speed.
      * The most significant frictional losses are proportional to 
      * speed and M_r can/should be very small if k_r is known. In order
      * to calculate k_r, data of more than one idle setpoint is needed. 
      * In case this data is not available, k_r=0 and M_r!=0.
      */
     double M_r;
        
     /**
      * This coefficient describes friction losses which are proportional 
      * to rotational speed. See M_r.
      */
     double k_r;
     
     /**
      * Motorkonstante [Nm/A]
      */
     double k_M;
     

     /**
      * throttle input, rate limited
      */
     CRRCMath::RateLimiter<double> throttle;
    
     /**
      * rate limit for throttle
      */
     double throttle_rate_max;
    
     /**
      * Do logging?
      */
     int nLog;        
  };
};
#endif
