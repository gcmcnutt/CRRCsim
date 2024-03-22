/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 *   Copyright (C) 2000, 2001 Jan Kansky (original author)
 *   Copyright (C) 2004-2010 Jan Reucker
 *   Copyright (C) 2004, 2005, 2006, 2008 Jens Wilhelm Wulf
 *   Copyright (C) 2005 Chris Bayley
 *   Copyright (C) 2005 Lionel Cailler
 *   Copyright (C) 2006 Todd Templeton
 *   Copyright (C) 2009 Joel Lienard
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

/** \file crrc_scenery.cpp
 *  This file defines a "scenery" class which contains all data
 *  and methods to construct and draw the landscape.
 */

//#define DRAW_SQUARES
#define DRAW_TRIANGLES
#include "../i18n.h"
#include "../include_gl.h"
#include <crrc_config.h>
#include "crrc_scenery.h"
#include "crrc_builtin_scenery.h"
#include "model_based_scenery.h"
#include "../crrc_main.h"
#include "../mod_misc/SimpleXMLTransfer.h"
#include "../mod_misc/filesystools.h"
#include "../mod_misc/ls_constants.h"
#include "../GUI/crrc_msgbox.h"


/** Load a scenery from a file
 *
 *  Loads the scenery described by the given scenery file.
 *  fname must contain the full path information and point
 *  to an XML scenery description file.
 *
 *  \param fname          scenery file (with full path)
 *  \param sky_variant    If the scenery file contains more than one sky definition,
 *                        which one has to be loaded?
 *  \return Pointer to new scenery on success, NULL on error
 */
Scenery* loadScenery(const char *fname, int sky_variant)
{
  Scenery* new_scenery = NULL;
  SimpleXMLTransfer* xml = NULL;
	
  // open waiting box : Display message during the execution of this function
  CGUIWaitingBox waitingbox(_("Scenery loading..."));
  Video::display();
  
  // try to open the specified file
  try
  {
    SimpleXMLTransfer* tag;
    xml = new SimpleXMLTransfer(fname);

    // Take a look at the version number of the config file.
    int nVer = xml->attributeAsInt("version", 1);
    if (nVer < 3)
    {
      throw (nVer);
    }

    // parse the <scene> tag to determine which kind of
    // subclass we need to set up
    tag = xml->getChild("scene");
    std::string type = tag->attribute("type", "not specified");

    if (type == "built-in")
    {
      // create one of the pre-defined sceneries
      std::string variant = tag->attribute("variant", "not specified");

      if (variant == "DAVIS")
      {
        new_scenery = new BuiltinSceneryDavis(xml, sky_variant);
      }
      else if (variant == "CAPE_COD")
      {
        new_scenery = new BuiltinSceneryCapeCod(xml, sky_variant);
      }
      else // "not specified" or other unknown variant
      {
        fprintf(stderr, "Unknown built-in variant %s in %s\n", variant.c_str(), fname);
        fprintf(stderr, "Defaulting to DAVIS\n");
        new_scenery = new BuiltinSceneryDavis(xml, sky_variant);
      }
    }
    else if (type == "model-based")
    {
      new_scenery = new ModelBasedScenery(xml, sky_variant);
    }
    else // "not specified" or other unknown type
    {
    }
  }
  catch (XMLException e)
  {
    std::string s = "XMLException: ";
    s += e.what();
    fprintf(stderr, "%s\n", s.c_str());
    new_scenery = NULL;
  }
  catch (int v)
  {
    std::string s = "Incompatible Scenery Version : ";
    fprintf(stderr, "%s%d\n", s.c_str(),v);
    new_scenery = NULL;
  }
  return new_scenery;
}


/**
 *  Constructor of the base class
 */
Scenery::Scenery(SimpleXMLTransfer *xml, int sky_variant)
    : name("unknown"), nSkyVariant(sky_variant)
{
  flDefaultWindSpeed = 0.0f;
  flDefaultWindDirection = 0.0f;
  ImposeWindDirection = false;
  wdDefaultTurbulence = 1.0f;
  OriginAltitude= 0.0f;
  xml_description = xml;

  if (xml != NULL)
  {
    SimpleXMLTransfer *tag;

    // generic scenery information
    name = xml->getChild("name", true)->getContentString();
    std::cout << "Loading scenery \"" << name << "\"" << std::endl;

    // player positions (view points)
    tag = xml->getChild("views");
    parsePositions(tag, views);

    // starting positions
    try
    {
      tag = xml->getChild("start");
    }
    catch (XMLException e)
    {
      tag = 0;
    }
    if (tag) parsePositions(tag, starts);

    // read real altitude of scene
    tag = xml->getChild("scene", true);
    OriginAltitude = tag->attributeAsDouble("altitude", 0.0f);
    //printf("--->real altitude: %f\n",OriginAltitude);
    
    // read default settings
    tag = xml->getChild("default.wind", true);
    flDefaultWindSpeed = tag->attributeAsDouble("velocity", 7.0f);
    flDefaultWindDirection = tag->attributeAsDouble("direction", 270.0f);
    wdDefaultTurbulence = tag->attributeAsDouble("turbulence", 1.0f);    

    if (cfgfile->getInt("video.enabled", 1))
    {
      SimpleXMLTransfer *sky;
    
      /// \todo error handling if creating the sky fails?
      int children = xml->getChildCount();
      std::vector<int> skies;
      for (int i = 0; i < children; i++)
      {
        if (xml->getChildAt(i)->getName() == "sky")
        {
          skies.push_back(i);
          std::cout << "  <sky> " << i << " at child idx " << i << std::endl;
        }
      }
      if (sky_variant < (int)skies.size())
      {
        std::cout << "  Using sky variant " << sky_variant << std::endl;
        sky = xml->getChildAt(skies[sky_variant]);
      }
      else
      {
        std::cout << "  Using first sky definition" << std::endl;
        sky = xml->getChild("sky", true);
        nSkyVariant = 0;
      }
    
      theSky = Video::setup_sky(sky);
    }
  }
}


/**
 *  Destructor of the base class
 */
Scenery::~Scenery()
{
if (xml_description != NULL)
  {
    delete xml_description;
  }
}


/**
 * Get pointeur on XML description section named "section_name"
 *
 */
SimpleXMLTransfer * Scenery::getXMLsection(const char *section_name)
{
  SimpleXMLTransfer * tag=0;;
  if(xml_description){
    try
      {
        tag = xml_description->getChild(section_name);
      }
    catch (XMLException e)
      {
      }
  }
return tag;
}


/**
 *  Look at the children of \c tag and read those with
 *  the name "position" into an array of positions.
 *
 *  \param tag node of the XML file containing the "position" children
 *  \param pa  reference to the position array
 *  \param default_on_empty add default 0/0/0 position if no position was
 *         found in the XML file?
 *  \return    number of positions added to the array
 */
int Scenery::parsePositions(SimpleXMLTransfer *tag, T_PosnArray& pa, bool default_on_empty)
{
  int ccount;       // child count
  int pcount = 0;   // position count

  ccount = tag->getChildCount();
  for (int i = 0; i < ccount; i++)
  {
    // parse all children of the <views> tag, accept only <position> tags
    SimpleXMLTransfer *view = tag->getChildAt(i);
    if (view->getName() == "position")
    {
      T_Position pos;
      pos.north = (-1)*view->attributeAsDouble("north", 0.0);
      pos.east  = view->attributeAsDouble("east", 0.0);
      pos.height = view->attributeAsDouble("height", 0.0);
      pos.name  = view->attribute("name", "no_name");
      pa.push_back(pos);
      pcount++;
    }
  }

  if ((pcount == 0) && (default_on_empty))
  {
    T_Position pos;
    pos.north     = 0.0;
    pos.east     = 0.0;
    pos.height     = 0.0;
    pos.name  = "empty_default";
    pa.push_back(pos);
    pcount++;
  }
  return pcount;
}


/**
 *  Get one of the specified player positions of the
 *  scenery.
 *
 *  \param num viewpoint index
 *  \return viewpoint coordinates
 *
 *  \todo right now, we always return position #0, because
 *        there's no way to select other positions.
 */
CRRCMath::Vector3 Scenery::getPlayerPosition(int num)
{
  double x,y,z;
  if(! views.empty())
  {
    x = views[0].east;
    y = views[0].height + getHeight(views[0].north,views[0].east); //////////////////////////
    z = views[0].north;
  }
  else 
  {
    x = y = z = 0;
  }

  return CRRCMath::Vector3(x, y, z);
}


/**
 *  Get the number of start positions of the
 *  scenery.
 */
int Scenery::getNumStartPosition()
{
  int num;
  num = starts.size();
  return num;
}


/**
 *  Get one of the specified start positions of the
 *  scenery.
 *
 *  \param num start point index
 *  \return point coordinates
 *
 */
CRRCMath::Vector3 Scenery::getStartPosition(int num)
{
  double x,y,z;
  if ( starts.size() !=0 )
  {
    x = starts[num].east;
    y = starts[num].height + getHeight(starts[num].north,starts[num].east);
    z = starts[num].north;
  }
  else x = y = z = 0;
  return CRRCMath::Vector3(x, y, z);
}


/**
 *  Get one of the specified start positions of the
 *  scenery.
 *
 *  \param string name of position
 *  \return point coordinates
 *
 */
CRRCMath::Vector3 Scenery::getStartPosition(std::string name)
{
  double x,y,z;
  for (unsigned int i=0;i < starts.size(); i++)
    {
    if( name == starts[i].name) return getStartPosition(i);
    }
  x = y = z = 0;
  return CRRCMath::Vector3(x, y, z);
}


/**
 *  Get name of the specified start positions of the
 *  scenery.
 *
 *  \param num start point index
 *  \return pointer on string name
 *
 */
std::string* const Scenery::getStartPositionName(int num)
{
  std::string *name;
  if ( starts.size() !=0 )
  {
   name = &(starts[num].name);
  }
  else name = NULL;
  return name;
}


/**
 *  Draw an array of wind vectors describing the wind field around model
 */
void Scenery::drawWindField(CRRCMath::Vector3 pos, int mode)
{
#define MOVING_ARRAY  0     // 0 or 1, option
#define NVECTORS      11    // number of vectors on a row, odd 
#define SPACING       75.   // distance between vectors, in ft
#define NPLANES       1     // number of vector planes, odd
#define SPACING_Z     50.   // distance between vector planes, in ft
#define MIN_HEIGHT    10.   // minimum altitude from terrain of wind vectors, in ft
#define VECTOR_SCALE  20.   // freestream wind vector length, in ft
#define RADIUS_SCALE  1.    // freestream wind vector base radius, in ft
#define SCALE_V_MIN   0.5   // lower end of wind speed color scale
#define SCALE_V_MAX   1.5   // higher end of wind speed color scale
#define SCALE_VZ_MIN -0.5   // lower end of vertical wind speed color scale
#define SCALE_VZ_MAX  0.5   // higher end of vertical wind speed color scale

  float X_rwy;
  float Y_rwy;
  float Z_rwy;
  if (MOVING_ARRAY)
  {
    // the array of vectors moves with the model
    X_rwy = pos.r[0];
    Y_rwy = pos.r[1];
    Z_rwy = pos.r[2];
  }
  else
  {
    // the array of vectors is fixed to the ground
    X_rwy = int(pos.r[0]/SPACING + (pos.r[0] >= 0 ? 0.5 : -0.5))*SPACING;
    Y_rwy = int(pos.r[1]/SPACING + (pos.r[1] >= 0 ? 0.5 : -0.5))*SPACING;
    Z_rwy = pos.r[2];
  }
  float flWindVel = cfg->wind->getVelocity(); // freestream wind
  
  float fade_distance = 0.6*NVECTORS*SPACING; // slightly more than half the array size
  
  GLUquadricObj *quadric = gluNewQuadric();
  ssgSimpleState *state_blend = new ssgSimpleState();
  state_blend->enable(GL_CULL_FACE);
  state_blend->disable(GL_COLOR_MATERIAL);
  state_blend->disable(GL_TEXTURE_2D);
  state_blend->disable(GL_LIGHTING);
  state_blend->enable(GL_BLEND);
  state_blend->apply();
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
  for (int j = 1; j <= NVECTORS; j++)
  {
    float y = Y_rwy + SPACING*(j - (NVECTORS+1)/2);
    
    for (int i = 1; i <= NVECTORS; i++)
    {
      float x = X_rwy + SPACING*(i - (NVECTORS+1)/2);
  
      for (int k = 1; k <= NPLANES; k++)
      {
        // Remember: z is negative upwards
        float z = Z_rwy - SPACING_Z*(k - (NPLANES+1)/2);
        float z_min = - (getHeight(x, y) + MIN_HEIGHT*k);
        z = z < z_min ? z : z_min;
        
        if (MOVING_ARRAY && (i == (NVECTORS+1)/2) && (j == (NVECTORS+1)/2) && (k == (NPLANES+1)/2))
        {
          // model position, don't draw wind vector
        }
        else
        {
          CRRCMath::Vector3 pnt(x, y, z);
          CRRCMath::Vector3 dist = pnt - pos;
          float distance = dist.length()/fade_distance;

          if (distance < 1.)
          {
            float alpha = 0.5*(1. - distance);
            
            CRRCMath::Vector3 vel;
            int err = calculate_wind(x, y, z, vel.r[0], vel.r[1], vel.r[2]);
            if (!err)
            {
              float velocity = vel.length()/flWindVel;
              float heading = -atan2(vel.r[1],vel.r[0]);
              float pitch = -atan2(vel.r[2],sqrt(vel.r[0]*vel.r[0] + vel.r[1]*vel.r[1]));
              
              glPushMatrix();
              glTranslatef(pnt.r[1], -pnt.r[2], -pnt.r[0]);
              glRotatef(SG_RADIANS_TO_DEGREES*heading, 0., 1., 0.);
              glRotatef(SG_RADIANS_TO_DEGREES*pitch,   1., 0., 0.);
              if (mode == 1)
                setColorFromScale(velocity, SCALE_V_MIN, SCALE_V_MAX, alpha);
              else
                setColorFromScale(-vel.r[2]/flWindVel, SCALE_VZ_MIN, SCALE_VZ_MAX, alpha);
              gluCylinder(quadric, 0., RADIUS_SCALE*velocity, VECTOR_SCALE*velocity, 8, 1);
              glPopMatrix();
            }
          }
        }
      }
    }
  }
  
  gluDeleteQuadric(quadric);
  delete state_blend;
}                               

void Scenery::setColorFromScale(float val, float min, float max, float alpha)
{
  //define a blue-green-red color scale from min to max
  
  float r = 0.;
  float g = 0.;
  float b = 0.;
  float range = max - min;
  float mid = 0.5*(max + min);
  float hue = (val - mid)/range;
  hue = hue < -0.5 ? -0.5 : hue > 0.5 ? 0.5 : hue;
  if (hue <= -0.25)
  {
    b = 1.;
    g = (hue + 0.5)/0.25;
  }
  else if (hue <= 0.)
  {
    b = - hue/0.25;
    g = 1.;
  }
  else if (hue <= 0.25)
  {
    g = 1.;
    r = hue/0.25;
  }
  else
  {
    g = - (hue - 0.5)/0.25;
    r = 1.;
  }
  glColor4f(r, g, b, alpha);  
}
