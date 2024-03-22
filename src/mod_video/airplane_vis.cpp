/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2009 Jan Reucker (original author)
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
  
/** \file airplane_vis.cpp
 * 
 *  This file is all about airplane visualization.
 */
#include "../i18n.h"
#include "airplane_vis.h"
#include "crrc_ssgutils.h"
#include "crrc_graphics.h"
#include "../mod_landscape/crrc_scenery.h"
#include <list>
#include <string>
#include <stdexcept>
#include <iostream>
#include <sstream>
#include "../GUI/crrc_msgbox.h"

#include "../global.h"  // only for LOG()


namespace Video
{
/// \todo there should be only one #define. Currently there are two
///       (inside and outside the namespace)
#define INVALID_AIRPLANE_VISUALIZATION -1

ssgRoot *ModelGraph = NULL ; // ssg graph for all model airplanes

std::vector<AirplaneVisualization*> AirplaneVisualization::ListOfVisualizations;


AirplaneVisualization::AirplaneVisualization( std::string const& model_name,
                                              std::string const& texture_path,
                                              CRRCMath::Vector3 const& pCG,
                                              SimpleXMLTransfer *xml)
{	
  ssgTexturePath(texture_path.c_str());
  // load model
  ssgEntity *model = ssgLoad(model_name.c_str());

  if (model != NULL)
  {
    // If it doesn't exists yet, create an SSG root node
    if (!ModelGraph)
      ModelGraph = new ssgRoot();
    
    // transformation matrix from SSG into CRRCsim's coordinate system
    sgMat4 it = {  {1.0,  0.0,  0.0,  0.0},
                   {0.0,  0.0, -1.0,  0.0},
                   {0.0,  1.0,  0.0,  0.0},
                   {(float)pCG.r[1],  (float)pCG.r[2],  (float)-pCG.r[0],  1.0}  };

    // Get half (approxiamtely) of the airplane model size    
    sphereRadius = model->getBSphere()->getRadius();
    
    // to be able to draw one model at a time, incapsulate
    // the model (and its shadow) in a selectable branch.
    // By default activate the model branch.
    model_branch = new ssgSelector(1);
    model_branch->select(1);
    ModelGraph->addKid(model_branch);
    model_trans = new ssgTransform();
    model_branch->addKid(model_trans);
    ssgTransform *initial_trans = new ssgTransform();
    model_trans->addKid(initial_trans);
    initial_trans->setTransform(it);

    // put the real object in a selectable branch
    // active by default
    object_branch = new ssgSelector(1);
    object_branch->setName("object_branch");
    object_branch->select(1);
    initial_trans->addKid(object_branch);
    object_branch->addKid(model);
#if (SHADOW_TYPE==SHADOW_VOLUME)
    // put the shadow volume object in a selectable branch
    // active by default
    shadow_branch = new ssgSelector(1);
    shadow_branch->setName("shadow_branch");
    shadow_branch->select(1);
    initial_trans->addKid(shadow_branch);
    shadow = new ShadowVolume(model);
    shadow_branch->addKid(shadow);
#endif

    // add animations ("real" model only, without shadow)
    initAnimations(xml, model);
  }
  else
  {
    std::string msg = "Error opening model ";
    msg += model_name;
    msg += " (texture path ";
    msg += texture_path;
    msg += ")";
    throw std::runtime_error(msg);
  }
}
  

AirplaneVisualization::~AirplaneVisualization()
{
  ssgBranch *parent = model_branch->getParent(0);
  parent->removeKid(model_branch);
}
  

void AirplaneVisualization::setPosition(CRRCMath::Vector3 const& pos,
                                        double phi, double theta, double psi)
{
  sgMat4 m;
  sgMat4 temp;
  sgVec3 rvec;
  
  sgMakeTransMat4(m, pos.r[1], -pos.r[2], -pos.r[0]);
  
  sgSetVec3(rvec, 0.0, 1.0, 0.0);
  sgMakeRotMat4(temp, 180.0f - (float)psi * SG_RADIANS_TO_DEGREES, rvec);
  sgPreMultMat4(m, temp);
  
  sgSetVec3(rvec, -1.0, 0.0, 0.0);
  sgMakeRotMat4(temp, (float)theta * SG_RADIANS_TO_DEGREES, rvec);
  sgPreMultMat4(m, temp);
  
  sgSetVec3(rvec, 0.0, 0.0, 1.0);
  sgMakeRotMat4(temp, (float)phi * SG_RADIANS_TO_DEGREES, rvec);
  sgPreMultMat4(m, temp);
  model_trans->setTransform(m);

#if (SHADOW_TYPE==SHADOW_VOLUME)
  shadow->update(m);
#endif
}


void AirplaneVisualization::resetShadow()
{
#if (SHADOW_TYPE==SHADOW_VOLUME)
  shadow->reset();
#endif
}


float AirplaneVisualization::getRadius()
{
  return sphereRadius;
}


/**
 * Create a new airplane visualization
 */
long new_visualization( std::string const& model_name,
                        std::string const& texture_path,
                        CRRCMath::Vector3 const& pCG,
                        SimpleXMLTransfer *xml)
{
  AirplaneVisualization* vis = NULL;
  long id = INVALID_AIRPLANE_VISUALIZATION;
  
  try
  {
    vis = new AirplaneVisualization(model_name, texture_path, pCG, xml);
    
    // add the new visualization to the list of all visualizations
    // first search for an empty entry
    std::vector<AirplaneVisualization*>::size_type pos;
    for ( pos = 0;
          pos < AirplaneVisualization::ListOfVisualizations.size();
          pos++)
    {
      if (AirplaneVisualization::ListOfVisualizations[pos] == NULL)
      {
        AirplaneVisualization::ListOfVisualizations[pos] = vis;
        id = (long)pos;
        break;
      }
    }
    
    // if no empty entry was found, just add it to the end of the list
    if (id == INVALID_AIRPLANE_VISUALIZATION)
    {
      AirplaneVisualization::ListOfVisualizations.push_back(vis);
      id = (long)(AirplaneVisualization::ListOfVisualizations.size() - 1);
    }
    
    std::ostringstream log;
    log << _("Loaded model ") << model_name << " (ID " << id << ")";
    LOG(log.str());
  }
  catch (std::runtime_error &e)
  {
    std::cerr << e.what() << std::endl;
    delete vis;
    vis = NULL;
    id = INVALID_AIRPLANE_VISUALIZATION;
  }
  
  return id;
}


/**
 * Deallocate an airplane visualization
 */
void delete_visualization(long id)
{
  if ((id >= 0) && (id < (long)AirplaneVisualization::ListOfVisualizations.size()))
  {
    delete AirplaneVisualization::ListOfVisualizations[id];
    AirplaneVisualization::ListOfVisualizations[id] = NULL;
  }
}


/**
 * Update the position of a visualization
 */
void set_position(long id,
                  CRRCMath::Vector3 const &pos,
                  double phi,
                  double theta,
                  double psi)
{
  if ((id >= 0) && (id < (long)AirplaneVisualization::ListOfVisualizations.size()))
  {
    AirplaneVisualization::ListOfVisualizations[id]->setPosition(pos, phi, theta, psi);
  }
}


/**
 * Reset the shadow of a visualization
 * e.g. due to a change of the scenery lights
 */
void reset_shadow(long id)
{
  if ((id >= 0) && (id < (long)AirplaneVisualization::ListOfVisualizations.size()))
  {
    AirplaneVisualization::ListOfVisualizations[id]->resetShadow();
  }
}


/**
 * Get half (approxiamtely) of the airplane size 
 * from the graphical model
 */
float get_radius(long id)
{
  if ((id >= 0) && (id < (long)AirplaneVisualization::ListOfVisualizations.size()))
  {
    return(AirplaneVisualization::ListOfVisualizations[id]->getRadius());
  }
  else
    return(0.0);
}


/**
 * Draw a single airplane visualisation
 * Either the real object or the shadow volume object is drawn
 */
void draw_airplane(long id, double current_time, bool fObject)
{
  if (ModelGraph)
  {
    // deactivate the visualisation branch of all models
    // except for the specifed one.
    std::vector<AirplaneVisualization*>::size_type pos;
    for ( pos = 0;
          pos < AirplaneVisualization::ListOfVisualizations.size();
          pos++)
    {
      if (AirplaneVisualization::ListOfVisualizations[pos] != NULL)
      {
        AirplaneVisualization::ListOfVisualizations[pos]->model_branch->select((long)pos == id);
        
        // activate either the real object or its shadow
        AirplaneVisualization::ListOfVisualizations[pos]->object_branch->select(fObject);
#if (SHADOW_TYPE==SHADOW_VOLUME)      
        AirplaneVisualization::ListOfVisualizations[pos]->shadow_branch->select(!fObject);
#endif
      }
    }
    
    // draw the airplane visualisation loaded in the graph
    ssgCullAndDraw(ModelGraph);
  }
}


/**
 * Draw all airplanes for which a visualisation has been created
 * Either the real objects or their shadow volume objects are drawn
 */
void draw_all_airplanes(double current_time, bool fObject)
{
  if (ModelGraph)
  {
    // activate all model visualisation branches
    std::vector<AirplaneVisualization*>::size_type pos;
    for ( pos = 0;
          pos < AirplaneVisualization::ListOfVisualizations.size();
          pos++)
    {
      if (AirplaneVisualization::ListOfVisualizations[pos] != NULL)
      {
        AirplaneVisualization::ListOfVisualizations[pos]->model_branch->select(1);
        
        // activate either the real object or its shadow
        AirplaneVisualization::ListOfVisualizations[pos]->object_branch->select(fObject);
#if (SHADOW_TYPE==SHADOW_VOLUME)      
        AirplaneVisualization::ListOfVisualizations[pos]->shadow_branch->select(!fObject);
#endif
      }
    }
    
    // draw all airplane visualisations loaded in the graph
    ssgCullAndDraw(ModelGraph);
  }
}  

} // end namespace Video::
