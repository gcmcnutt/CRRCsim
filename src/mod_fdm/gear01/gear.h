/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2008 - Jan Reucker (refactoring, see note below)
 *
 * This file is partially based on work by
 *   Jan Kansky
 *   Jens Wilhelm Wulf
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
/** \file gear.h
 *
 *  Declaration of classes for hardpoint/wheel management
 */

#ifndef FDM_WHEELS_H
#define FDM_WHEELS_H

# include <vector>
# include "../ls_types.h"
# include "../fdm_inputs.h"
# include "../../mod_math/vector3.h"
# include "../../mod_math/matrix33.h"
# include "../../mod_math/matrix44.h"
# include "../../mod_misc/SimpleXMLTransfer.h"
# include "../fdm_env.h"
//# include "../../mod_misc/lib_conversions.h"

/** A base class for hard point transformations
 *
 */
class HardPointTransformation
{
  public:
    HardPointTransformation()
    {
    }
    
    virtual ~HardPointTransformation()
    {
    }
  
    virtual void transform(CRRCMath::Vector3& hp) = 0;
    virtual void update() = 0;
};


/** Rotary transformation for a hard point
 *
 */
class HardPointRotation : public HardPointTransformation
{
  public:
    HardPointRotation(SimpleXMLTransfer *xml, TSimInputs const& in);
    void transform(CRRCMath::Vector3& hp);
    void update();
  
  private:
    std::string symbolic_name;
    float max_angle_rad;
    float abs_max_angle_rad;
    float fallback_data;
    CRRCMath::Vector3 hinge[2];
    CRRCMath::Vector3 axis;
    CRRCMath::Matrix44f move_orig;
    CRRCMath::Matrix44f move_back;
    CRRCMath::Matrix44f xform;
    std::vector<const float*> datasource;
    std::vector<float>  source_gain;

};


class CRRCAnimation;
class WheelSystem;

/**
* This class holds information about a single hard point/wheel
* on the airplane.
*/
class Wheel
{
  friend class WheelSystem;

  public:
    Wheel(const WheelSystem* ws);
    
    void update(FDMEnviroment*      env,
                CRRCMath::Matrix33  LocalToBody,
                CRRCMath::Vector3   const& v_P_CG_Rwy,
                CRRCMath::Vector3   const& v_R_omega_body,
                CRRCMath::Vector3   const& v_V_local_rel_ground,
                SCALAR psi);
    CRRCMath::Vector3 tempF, tempM;
 
  private:
    /** An arbitrary ID assigned by the WheelSystem.
     *  For debugging only.
     */
    unsigned int nID;

    /** The wheel system we belong to */
    const WheelSystem * myWheelSystem;
    /**
     * position in body axes: x,y,z relative to center of gravity
     */
    CRRCMath::Vector3 v_P;

    std::string   anim_name;
    HardPointTransformation *hpt;

    double spring_constant;
    double spring_damping;
  
    double max_force;

    TSimInputs::eSteeringMap steering_mapping;   ///<  Indicates which RC channel controls the steering
    double steering_max_angle; ///<  Indicates maximum angle of steering wheel
    double percent_brake;
    double caster_angle_rad;   ///<  Alignment of the wheel
};


/**
 * This class represents a system of hardpoints (wheels).
 */
class WheelSystem
{
  friend class Wheel;

  public:
    WheelSystem();
    void init(SimpleXMLTransfer *ModelFile, SCALAR def_span);

    void update(TSimInputs* inputs,
                FDMEnviroment* env,
                CRRCMath::Matrix33 LocalToBody,
                CRRCMath::Vector3  const& v_P_CG_Rwy,
                CRRCMath::Vector3  const& v_R_omega_body,
                CRRCMath::Vector3  const& v_V_local_rel_ground,
                SCALAR psi);
    CRRCMath::Vector3 getForces() const {return v_Forces;};
    CRRCMath::Vector3 getMoments() const {return v_Moments;};

   /**
    * Wingspan of the aircraft in feet
    */
    double getWingspan() const { return(span_ft); };
   
   /**
    * returns Z coordinate of lowest point
    */
    double getZLow() const { return(dZLow); };

   /**
    * returns Z coordinate of highest point
    */
    double getZHigh() const { return(dZHigh); };

  private:
    std::vector<Wheel>  wheels;
    CRRCMath::Vector3   v_Forces;
    CRRCMath::Vector3   v_Moments;

   /**
    * Wingspan in feet (calculated from hardpoints)
    */
   double span_ft;

   /**
    * Z coordinate of lowest point
    */
   double dZLow;
   
   /**
    * Z coordinate of highest point
    */
   double dZHigh;
   
   /**
    * A local copy of the current sim inputs
    */
   TSimInputs wheel_inputs;
   
};





#endif  // FDM_WHEELS_H
