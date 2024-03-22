/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2006, 2008, 2009 - Jens Wilhelm Wulf (original author)
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
#ifndef BATTERY_H
# define BATTERY_H

# include <vector>
# include "../../mod_misc/SimpleXMLTransfer.h"
# include "../../mod_math/pt1.h"
# include "shaft.h"
# include "values_step.h"

namespace Power
{

  /**
   * This class is part of the power system. To simply use the system, you should not
   * access or call any of its members. Please take a look at Power instead.
   *
   * This class models a battery.
   *
   * A battery has an initial capacity. While current is being drawn from the battery, the capacity
   * left becomes smaller. Once nothing is left, its voltage (as seen in PowerValuesStep passed in step)
   * is set to zero. The battery is definitely empty in this case.
   *
   * Apart from this behaviour, the voltage is modelled as
   *  <tt>U = U_0(C, C_0) - R_I * I</tt>. \c I is the current drawn from the battery.
   *
   * The no-load-voltage \c U_0 is a function of the capacity left. It is read from a table.
   *
   * If the battery voltage drops below \c U_off, the voltage is set to zero and locked. By setting the throttle
   * command to zero, it can be unlocked again. This is similar to what speed controllers do.
   * 
   * Set <tt>throttle_min</tt> to &gt; 0 to model a glow engine which can only be started once 
   * and runs at that minimum throttle afterwards.
   * 
   * The xml configuration of a battery looks like this:
   \verbatim
   <battery C="1.2" U_0="9.6" U_off="7" R_I="10E-3" throttle_min="0">
     <U_0rel>
       1.05;
       0.95;
       0.90;        
       0.85;
       0.85;        
       0.85;        
       0.85;        
       0.85;        
       0.85;        
       0.80;        
       0.75;        
       0.70;
     </U_0rel>
     ...shafts connected to this battery...
   </battery>
   \endverbatim
   * 
   * In contrast to other values in the power system, the initial capacity \c C is given in Ah, not As.
   *
   * It is possible to read the parameters of a battery from a separate file. In this case use
   * something like
   \verbatim
   <battery filename="nicd12_30" throttle_min="0">
   \endverbatim
   * instead of writing down the parameters directly. The system will try to load a file 
   * <tt>./models/battery/nicd12_30.xml</tt> which might look like this:
   \verbatim
   <?xml version="1.0"?>

   <battery C="3.0" U_0="12" U_off="9" R_I="10E-3">
     <U_0rel>
       1.05;
       0.95;
       0.90;        
       0.85;
       0.85;        
       0.85;        
       0.85;        
       0.85;        
       0.85;        
       0.80;        
       0.75;        
       0.70;
     </U_0rel>
   </battery>
   \endverbatim
   *
   * In both cases, the section \c U_0rel is a table showing the no-load-voltage \c U_0 as a function of the capacity left.
   * The table does not contain absolute values. In this example, \c U_0 at full charge is <tt>1.05 * 12 V</tt>. It does not
   * matter how many entries this table contains; they are assumed to be at equal distances as far as the capacity left is 
   * concerned.
   *
   * @author Jens Wilhelm Wulf
   */
  class Battery
  {
    public:

     /**
      * only used for automagic construction
      */
     Battery();

     /**
      * Read configuration from xml and create elements below.
      * @param xml
      */
     Battery(SimpleXMLTransfer* xml);
       
     /**
      * deallocate battery ressources
      */
     ~Battery();

     /**
      * Go ahead values->dt seconds in the simulation. Calls Shaft::step() for each shaft in the list.
      *
      * @param values relevant data for a step
      */
     void step(PowerValuesStep* values);

     /**
      * prints remaining capacity to std:cout
      */
     void showCapacity();

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
     
    private:

     /**
      * Capacity at full charge in As.
      */
     double C_0;

     /**
      * Resistance in Ohm.
      */
     double R_I;

     /**
      * Voltage at Capacity [V]
      */
     std::vector<double> voltage;

     /**
      * Voltage below which all the connected engines are turned off [V].
      */
     double U_off;

     /**
      * Remaining capacity in As.
      */
     CRRCMath::Integrationsverfahren<double> C;

     /**
      * voltage [V]
      */
     double U;

     /**
      * Lower limit for throttle input. Set to >0 if you want a behaviour of 
      * a piston engine: once started, it keeps running with at least that throttle.
      * Set to zero otherwise. 
      * This is an attribute of Battery (instead of engine) because of the brake in shaft.
      */
     double throttle_min;
     
     /**
      * 
      */
     double throttle_old;
     
     /**
      * Shafts driven by the power of this battery.
      */
     std::vector<Shaft*> shafts;

     /**
      * State of low voltage cut off
      */
     int nUOffStatus;
     
     /**
      * conversion factor for easy interpolation
      */
     double dInterpFact;     
  };

}
#endif
