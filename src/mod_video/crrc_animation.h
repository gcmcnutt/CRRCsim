/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2006-2009 Jan Reucker (original author)
 * Copyright (C) 2008 Jens Wilhelm Wulf
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
  

/** \file crrc_animation.h
 *
 *  Classes for animated model parts.
 */

#ifndef CRRC_ANIMATION_H
#define CRRC_ANIMATION_H

#include <vector>

#include <plib/ssg.h>
#include "../mod_main/EventDispatcher.h"
#include "../mod_misc/SimpleXMLTransfer.h"
#include "../mod_fdm/fdm_inputs.h"
#include "../mod_math/vector3.h"

// --- forward declaration -----------
class CRRCAnimation;

// --- non-class-functions -----------

namespace Video
{
void initAnimations(SimpleXMLTransfer *model_file, ssgEntity* model);
} // end namespace Video::


/** A base class for all animations
 *
 *  This is the ABC for all animations, providing a
 *  unified interface to the scene rendering engine.
 */
class CRRCAnimation : public ssgBase
{
  public:
    /// Constructor
    CRRCAnimation(ssgBranch *branch);
  
    /// Destructor
    virtual ~CRRCAnimation();
 
    /// Init the animation
    virtual void init() = 0;

    /// Update the animation before rendering
    virtual int update() = 0;
  
    /// get-method for the animation branch
    ssgBranch* getBranch()  {return anim_branch;};

    /// get-method for the animation's symbolic name
    std::string getSymbolicName() const {return symbolic_name;};
  
    /// Transform an arbitrary point in body coordinates
    virtual void transformPoint(CRRCMath::Vector3& point) {};

  protected:
    ssgBranch *anim_branch;
    std::string symbolic_name;
};


/** Rotate a control surface
 *
 *  This animation rotates a control surface according to
 *  the control input.
 */
class CRRCControlSurfaceAnimation : public CRRCAnimation
{
  public:
    /// Constructor
    CRRCControlSurfaceAnimation(SimpleXMLTransfer *xml);
  
    /// Destructor
    ~CRRCControlSurfaceAnimation();
  
    /// Initialize the animation (virtual hook function)
    void init();
  
    /// Initialize the animation (this one does the real work)
    void realInit();
  
    /// Update the animation before rendering
    int update();

    /// Transform an arbitrary point in body coordinates
    void transformPoint(CRRCMath::Vector3& point);
  
    /// Callback for receiving control input values
    void axisValueCallback(const Event* ev);
  
  private:
    std::vector<float*> datasource;
    std::vector<float>  source_gain;
    float fallback_data;
    sgVec3 hinge_1;
    sgVec3 hinge_2;
    sgVec3 axis;
    sgMat4 move_to_origin;
    sgMat4 move_back;
    float  max_angle;
    float  abs_max_angle;
    sgMat4 current_transformation;
    EventAdapter<CRRCControlSurfaceAnimation> eventAdapter;
  
    float aileron;
    float elevator;
    float rudder;
    float throttle;
    float spoiler;
    float flap;
    float retract;
    float pitch;
};


#endif // CRRC_ANIMATION_H
