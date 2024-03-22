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
  

/** \file crrc_animation.cpp
 *
 *  Classes for animated 3D model parts.
 */

#include "crrc_animation.h"
#include "crrc_ssgutils.h"


// --- non-class-functions ------------------------------------------

/**
 *  Callback to update an animation.
 *
 *  This callback is activated whenever an animation node is
 *  traversed while rendering the scenegraph. It simply calls
 *  the animation's \c update() method which should know how
 *  to update itself.
 *
 *  This method was taken from SimGear, credits go to the original
 *  authors.
 *
 *  \param entity   the entity for which the callback was activated
 *  \param mask     dunno
 *  \return 1 if the animation was updated
 */
static int animation_callback (ssgEntity * entity, int mask)
{
    return ((CRRCAnimation *)entity->getUserData())->update();
}


namespace Video
{

void createAnimation(SimpleXMLTransfer *animation, ssgEntity* model)
{
  ssgEntity *node;
  
  if (animation->getName() != "animation")
  {
    std::cerr << "creatAnimation: invalid child ";
    std::cerr << animation->getName() << std::endl;
  }
  else
  {
    std::string node_name = animation->getString("object.name", "default");
    std::string type      = animation->getString("type", "default");

    node = SSGUtil::findNamedNode(model, node_name.c_str());
    if (node != NULL)
    {
      CRRCAnimation *anim = NULL;
      std::cout << "createAnimation: found animation node ";
      std::cout << node_name << ", type " << type << std::endl; 
      
      if (type == "ControlSurface")
      {
        anim = new CRRCControlSurfaceAnimation(animation);
      }
      else
      {
        std::cerr << "createAnimation: unknown animation type \'"
                  << type << "\'" << std::endl;
      }
      
      if (anim != NULL)
      {
        if (anim->getBranch() == NULL)
        {
          std::cerr << "createAnimation: defunct animation class (animation branch is <NULL>)\n";
          exit(EXIT_FAILURE);
        }
        else
        {
          SSGUtil::spliceBranch(anim->getBranch(), node);
          anim->init();
          anim->setName("Animation");
          anim->getBranch()->setUserData(anim);
          anim->getBranch()->setTravCallback(SSG_CALLBACK_PRETRAV, animation_callback);
        }
      }
      
    }
    else
    {
      std::cerr << "createAnimation: node \'" << node_name << "\' not found in 3D model" << std::endl;
    }
  }
}


/** \brief Add animations to a model
 *
 *  This method reads animation description tags from a model file
 *  and tries to add the corresponding animations to the 3D model.
 *
 *  \todo Right now there's only one type of animation: movable control
 *  surfaces. Therefore this method receives a pointer to the control
 *  input class. If animations are added that need a different kind of
 *  input for their update() method, we need to decide how to pass all
 *  this stuff to initAnimations().
 *
 *  \param  model_file    XML model description file
 *  \param  model         scenegraph of the 3D model
 */
void initAnimations(SimpleXMLTransfer *model_file, ssgEntity* model)
{
  SimpleXMLTransfer *animations = model_file->getChild("animations", true);
  int num_anims = animations->getChildCount();
  std::cout << "initAnimations: found " << num_anims << " children" << std::endl;
  
  for (int i = 0; i < num_anims; i++)
  {
    SimpleXMLTransfer *animation = animations->getChildAt(i);
    createAnimation(animation, model);
  }
}

} // end namespace Video::



// --- Base class -----------------------------------------

/**
 *  Create a CRRCAnimation object.
 *
 */
CRRCAnimation::CRRCAnimation(ssgBranch *branch)
  : anim_branch(branch), symbolic_name("no_name_set")
{
  branch->recalcBSphere();
}


/**
 *  Delete a CRRCAnimation object.
 *
 */
CRRCAnimation::~CRRCAnimation()
{
}



// --- Control surface animation ------------------------------------

/**
 *  Create a CRRCControlSurfaceAnimation object
 *
 *  Initialize the animation from an 
 *  <animation type="ControlSurface"> tag
 */
CRRCControlSurfaceAnimation::CRRCControlSurfaceAnimation(SimpleXMLTransfer *xml)
 : CRRCAnimation(new ssgTransform()), fallback_data(0.0f),
   eventAdapter(this, &CRRCControlSurfaceAnimation::axisValueCallback, Event::Input),
    aileron(0.0f), elevator(0.0f), rudder(0.0f), throttle(0.0f),
    spoiler(0.0f), flap(0.0f), retract(0.0f), pitch(0.0f)
{
  bool failed = false;
  
  // evaluate <object> tag
  SimpleXMLTransfer *map = xml->getChild("object", true);
  symbolic_name = map->getString("name", "no_name_set");
  max_angle = (float)(map->getDouble("max_angle", 0.0) * SG_RADIANS_TO_DEGREES);
  abs_max_angle = (float)fabs((double)max_angle);

  // find hinges and evaluate all <control> tags
  int num_controls = 0;
  int num_hinges = 0;
  for (int i = 0; i < xml->getChildCount(); i++)
  {
    SimpleXMLTransfer *child = xml->getChildAt(i);
    if (child->getName() == "hinge")
    {
      // found a <hinge> child
      sgVec3 pos;
      pos[SG_X] = (float)(-1 * child->getDouble("y", 0.0));
      pos[SG_Y] = (float)(-1 * child->getDouble("x", 0.0));
      pos[SG_Z] = (float)(-1 * child->getDouble("z", 0.0));
      if (num_hinges == 0)
      {
        sgCopyVec3(hinge_1, pos);
      }
      else if (num_hinges == 1)
      {
        sgCopyVec3(hinge_2, pos);
      }
      num_hinges++;
    }
    else if (child->getName() == "control")
    {
      // found a <control> child
      // The "*2" factor for each gain value scales the control input
      // values from -0.5...+0.5 to -1.0...+1.0. This saves one
      // float multiplication per mapping in the runtime update() routine.
      // NB: unsigned control (throttle, spoiler, retract) are not scaled,
      //     since control input is already in range 0.0...+1.0
      std::string mapping = child->getString("mapping", "NOTHING");
      float gain = (float)child->getDouble("gain", 1.0);
      if (mapping == "ELEVATOR")
      {
        datasource.push_back(&elevator);
        source_gain.push_back(gain * 2);
        num_controls++;
      }
      else if (mapping == "AILERON")
      {
        datasource.push_back(&aileron);
        source_gain.push_back(gain * 2);
        num_controls++;
      }
      else if (mapping == "THROTTLE")
      {
        datasource.push_back(&throttle);
        source_gain.push_back(gain);
        num_controls++;
      }
      else if (mapping == "RUDDER")
      {
        datasource.push_back(&rudder);
        source_gain.push_back(gain * 2);
        num_controls++;
      }
      else if (mapping == "FLAP")
      {
        datasource.push_back(&flap);
        source_gain.push_back(gain * 2);
        num_controls++;
      }
      else if (mapping == "SPOILER")
      {
        datasource.push_back(&spoiler);
        source_gain.push_back(gain);
        num_controls++;
      }
      else if (mapping == "RETRACT")
      {
        datasource.push_back(&retract);
        source_gain.push_back(gain);
        num_controls++;
      }
      else if (mapping == "PITCH")
      {
        datasource.push_back(&pitch);
        source_gain.push_back(gain * 2);
        num_controls++;
      }
      else
      {
        std::cerr << "ControlSurfaceAnimation: ignoring <control> tag without mapping." << std::endl;
      }
      
    }
  }

  if (num_controls < 1)
  {
    std::cerr << "ControlSurfaceAnimation: found animation without proper <control> tag. Animation disabled." << std::endl;
    failed = true;
  }

  if (num_hinges < 2)
  {
    std::cerr << "ControlSurfaceAnimation: Must specify exactly two hinges!" << std::endl;
    failed = true;
  }
  else
  {
    if (num_hinges > 2)
    {
      std::cerr << "ControlSurfaceAnimation: Must specify exactly two hinges!" << std::endl;
      std::cerr << "ControlSurfaceAnimation: Ignoring excessive hinge tag(s)." << std::endl;
    }
    sgSubVec3(axis, hinge_2, hinge_1);
    if (sgLengthVec3(axis) < 0.001)
    {
      std::cerr << "ControlSurfaceAnimation: Insufficient spacing between hinges!" << std::endl;
      failed = true;
    }
  }

  if (failed)
  {
    std::cerr << "ControlSurfaceAnimation: Animation setup failed." << std::endl;
    // set to some non-critical defaults
    datasource.resize(1);
    datasource[0] = &fallback_data;
    source_gain.resize(1);
    source_gain[0] = 1.0;
    sgSetVec3(hinge_1, 0.0f, 0.0f, 0.0f);
    sgSetVec3(hinge_2, 1.0f, 0.0f, 0.0f);
    sgSubVec3(axis, hinge_2, hinge_1);
  }
  
  sgMakeIdentMat4(move_to_origin);
  move_to_origin[3][0] = -hinge_1[0];
  move_to_origin[3][1] = -hinge_1[1];
  move_to_origin[3][2] = -hinge_1[2];

  sgMakeTransMat4(move_back, hinge_1);

  realInit();
}


/**
 *  Delete a CRRCControlSurfaceAnimation object
 */
CRRCControlSurfaceAnimation::~CRRCControlSurfaceAnimation()
{
}


/**
 *  Initialize a CRRCControlSurfaceAnimation object
 */
void CRRCControlSurfaceAnimation::realInit()
{
  sgQuat q;
  sgAngleAxisToQuat(q, 0.0f, axis);
  sgQuatToMatrix(current_transformation, q);

  ((ssgTransform*)anim_branch)->setTransform(current_transformation);
}

/**
 *  The "official" virtual init()
 */
void CRRCControlSurfaceAnimation::init()
{
  realInit();
}


/**
 *  Update the animation. This method will be called by the
 *  scenegraph rendering function just before the animated
 *  branch will be rendered.
 */
int CRRCControlSurfaceAnimation::update()
{
  // calculate the deflection angle by summing up the contribution
  // of all mapped control inputs times the individual gain
  float angle = 0.0f;
  for (int i = 0; i < (int)datasource.size(); i++)
  {
    angle += *datasource[i] * source_gain[i] * max_angle;
  }
  // limit to max_angle (caution: max_angle could be < 0.0, so we use the abs value!)
  if (angle > abs_max_angle)
  {
    angle = abs_max_angle;
  }
  else if (angle < -abs_max_angle)
  {
    angle = -abs_max_angle;
  }

  // calculate transformation matrix for surface rotation
  sgQuat q;
  sgMat4 qmat;
  sgAngleAxisToQuat(q, angle, axis);
  sgQuatToMatrix(qmat, q);

  sgMultMat4(current_transformation, qmat, move_to_origin);
  sgPostMultMat4(current_transformation, move_back);
  
  ((ssgTransform*)anim_branch)->setTransform(current_transformation);
  return 1;
}


void CRRCControlSurfaceAnimation::transformPoint(CRRCMath::Vector3& point)
{
  sgVec3 pnt_sg;

  sgSetVec3(pnt_sg, (float)point.r[0], (float)point.r[1], (float)point.r[2]);
  sgXformVec3(pnt_sg, current_transformation);

  point.r[0] = pnt_sg[SG_X];
  point.r[1] = pnt_sg[SG_Y];
  point.r[2] = pnt_sg[SG_Z];
}


void CRRCControlSurfaceAnimation::axisValueCallback(const Event* ev)
{
  const AxisUpdateEvent* aue = dynamic_cast<const AxisUpdateEvent*>(ev);
  
  if (aue != NULL)
  {
    aileron = aue->getAileron();
    elevator = aue->getElevator();
    rudder = aue->getRudder();
    throttle = aue->getThrottle();
    flap = aue->getFlap();
    spoiler = aue->getSpoiler();
    retract = aue->getRetract();
    pitch = aue->getPitch();
  }
}
