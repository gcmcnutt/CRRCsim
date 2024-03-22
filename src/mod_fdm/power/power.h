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
#ifndef POWER_H
# define POWER_H

# include <vector>
# include "../../mod_misc/SimpleXMLTransfer.h"
# include "../../mod_math/vector3.h"
# include "battery.h"

/** \brief power/propulsion system
 * 
 * The power/propulsion system is packed together in a namespace. Users of the system should only 
 * use Power::Power, which does manage everything.
 */
namespace Power
{
  
  /**
   *
   * This class manages everything related to battery, engine, propeller.
   * Throughout this class (and the classes used by it) the SI unit system
   * is used (with a small number of exceptions).
   * 
   * Values used are:
   *   - lengths \c H and \c D in m
   *   - rotational velocity \c omega in rad/s
   *   - rotational velocity \c n in 1/s
   *   - linear velocity \c V in m/s
   *   - Torque \c M in Nm
   *   - Force/thrust \c F in N
   *   - Inertia \c J in kg m^2
   *   - Resistance \c R in Ohm
   *   - Current \c I in A
   *   - Voltage \c U in V
   *   - Capacity \c C in Ah
   * 
   * One airplane needs exactly one Power system, even if the latter is empty. 
   * The Power system for one airplane can contain the following items, which are organized in 
   * a tree-like structure which represents mechanical and electrical connections.
   *   - Power::Battery \n
   *     At least one is needed, unless you have a glider.
   *   - Power::Shaft \n
   *     At least one is needed.
   *     Every engine/propeller is (mechanically) connected to a shaft.
   *     One or more shafts are connected to a battery. This is what electrically relates
   *     engines to batteries.
   *   - Power::Engine_DCM \n
   *     One or more engines can be connected to a shaft.
   *   - Power::Propeller or Power::SimpleThrust \n
   *     One or more of them can be connected to a shaft.
   *
   * 
   * This is an example setup (without gearboxes):
   * 
   *  \dot
   digraph ExamplePowerSystem {
     rankdir=BT;

     power [shape = box];
       battery1 [shape = box];
       battery1 -> power;
         shaft1_1 [shape = box];
         shaft1_1 -> battery1;
           engine1_1_1 [shape = box];
           engine1_1_1 -> shaft1_1;
           propeller1_1_2 [shape = box];
           propeller1_1_2 -> shaft1_1;
       battery2 [shape = box];
       battery2 -> power;
         shaft2_1 [shape = box];
         shaft2_1 -> battery2;
           engine2_1_1 [shape = box];
           engine2_1_1 -> shaft2_1;
           propeller2_1_2 [shape = box];
           propeller2_1_2 -> shaft2_1;
           propeller2_1_3 [shape = box];
           propeller2_1_3 -> shaft2_1;
         shaft2_2 [shape = box];
         shaft2_2 -> battery2;
           engine2_2_1 [shape = box];
           engine2_2_1 -> shaft2_2;
           propeller2_2_2 [shape = box];
           propeller2_2_2 -> shaft2_2;
   }
   * \enddot
   * 
   * The system is configured using a description in xml. The connections mentioned above are given by the structure of xml elements.
   * Here's an example of a system with one
   * battery; there is a gearbox with two engines driving one prop.
   \verbatim
   <power>
     <battery filename="nicd12_30" throttle_min="0">
       <shaft J="2E-6" brake="0">
         <engine filename="astro_cobalt_10">
           <gearing J="0" i="2.5" />
         </engine>
         <engine filename="astro_cobalt_10">
           <gearing J="0" i="2.5" />
         </engine>
         <propeller D="0.3" H="0.2" J="3E-6" n_fold="-1" downthrust="2" />
       </shaft>
     </battery>    
   </power>
   \endverbatim
   * 
   * Take a look at the individual items for details of their description.
   * 
   * If you don't want to define every single item of the system, you can take the easy way.
   * Use a description like this:
   \verbatim
     <automagic F="12" V="15.8">
      <battery throttle_min="0">
        <automagic T="420" />
        <shaft J="0" brake="1">
          <propeller D="0.243" H="0.17" J="0" n_fold="5" downthrust="2" />
          <engine>
            <automagic omega_p="2827" eta_opt="0.78" eta="0.7" />
          </engine>
        </shaft>
      </battery>
    </automagic>
   \endverbatim
   * 
   * This will create a power system which delivers a thrust of <tt>F=12 N</tt> at a velocity of <tt>V=15.8 m/s</tt>.
   * The battery will be designed to last <tt>T=420 s</tt>.
   * You also need to fill in the dimensions of the propeller. It is best to take everything else as shown in the example.
   * 
   * If you use this \e automagic way of creating the system, the program will output the configuration it calculated from your
   * values to give you a starting point for fine grained tunings.
   * 
   * @author Jens Wilhelm Wulf
   */
  class Power
  {
    public:

     /**
      * Creates an empty power system
      */
     Power();

     /**
      * Tries to load attributes of the power system. This is the only valid
      * way of creating it!
      * Throws an exception on error.
      *
      * @param xml Contains the configuration options of the system. See ./documentation/power.htm
      */
     Power(SimpleXMLTransfer* xml, int nVerbosity = 3);
     
     /**
      * Deallocates a power system.
      */
     ~Power();
    
     /**
      * Tries to reload parameters of the power system. Only parameters are allowed to have
      * changed since its creation, but not its structure.
      * The systems state will not change during this call, so this makes it possible to
      * change parameters while the simulation is running.
      * Throws an exception on error.
      *
      * @param xml Contains the configuration options of the system. See ./documentation/power.htm
      */
     void ReloadParams(SimpleXMLTransfer* xml, int nVerbosity = 3);
    
     /**
      * Go ahead dt seconds in the simulation. Calls Battery::step() for each battery in the list.
      * Note that force and moment must be initialized (to zero or something else) before calling this!
      *
      * @param dt      timestep [s]
      * @param inputs  pointer to inputs
      * @param VRelAir Velocity of power system relative to airmass, [m/s].
      * @param force   Force created by power system is added to this vector [N] (body axes).
      * @param moment  Torque created by power system is added to this vector [Nm] (body axes).
      */
     void step(double                dt,
               TSimInputs*           inputs,
               CRRCMath::Vector3     VRelAir,
               CRRCMath::Vector3*    force,
               CRRCMath::Vector3*    moment);
     
     /**
      * Returns revolutions per second of a propeller. If there is more than one,
      * the one of the last in the list is used [1/s].
      */
     double getPropFreq() const { return(dPropFreq); };
     
     /**
      * Returns lowest relative battery capacity left (0..1).
      */
     double getBatteryMin() const { return(dBatCapLeftMin); };
    
     /**
      * Returns average battery voltage
      */
     double GetVoltageAvg() const { return(dVoltageAvg); };
             
     /**
      * Resets battery status, initialize states
      */
     void InitStates(CRRCMath::Vector3 vInitialVelocity);

     /**
      * Returns throttle needed to deliver a certain force at a 
      * certain pitch setting.
      * Please note that this causes a simulation, changing the state
      * of the power system. Calculating this directly would add a lot 
      * of complexity to the power system or make it less flexible. 
      */
     float Sim_GetThrottle(CRRCMath::Vector3  VRelAir, float force, float pitch, CRRCMath::Vector3& torque);
    
     /**
      * Returns pitch needed to deliver a certain force at a 
      * certain throttle setting.
      * Please note that this causes a simulation, changing the state
      * of the power system. Calculating this directly would add a lot 
      * of complexity to the power system or make it less flexible. 
      */
     float Sim_GetPitch(CRRCMath::Vector3  VRelAir, float force, float throttle, CRRCMath::Vector3& torque);

     void Sim_UntilStable(TSimInputs*        inputs,
                          CRRCMath::Vector3  VRelAir,
                          double             lim,
                          CRRCMath::Vector3* force,
                          CRRCMath::Vector3* moment);
      
   private:
     
     /**
      * List of batteries in use.
      */
     std::vector<Battery*> batteries;

     /**
      * Rotational speed of the last propeller calculated [1/s].
      */
     double dPropFreq;
     
     /**
      * Lowest relative battery capacity left (0..1).
      */
     double dBatCapLeftMin;

     double dVoltageAvg;
  };
  
}

#endif
