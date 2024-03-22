/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2000, 2001 Jan Edward Kansky (original author)
 * Copyright (C) 2005-2009 Jan Reucker
 * Copyright (C) 2005, 2006, 2008 Jens Wilhelm Wulf
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
  
/** \file crrc_loadair.cpp
 * 
 *  The audio-visual representation of an airplane model.
 */

#include "crrc_loadair.h"

#include <fstream>
#include <iostream>
#include <math.h>

#include "global.h"
#include "SimStateHandler.h"
#include "mod_misc/filesystools.h"
#include "crrc_sound.h"
#include "mod_misc/ls_constants.h"
#include "crrc_main.h"
#include "mod_fdm/xmlmodelfile.h"


CRRCAirplane::CRRCAirplane()
{
}

CRRCAirplane::~CRRCAirplane()
{
  for (int i = 0; i < (int)sound.size(); i++)
  {
    Global::soundserver->stopChannel(sound[i]->getChannel());
    delete sound[i];
  }
}


/*****************************************************************************/

CRRCAirplaneV2::CRRCAirplaneV2()
{
}


CRRCAirplaneV2::CRRCAirplaneV2(SimpleXMLTransfer* xml)
{
  printf("CRRCAirplaneV2(xml)\n");

  // initialize the airplane's sound
  initSound(xml);    

  // initialize the visual representation
  // first collect all relevant information from the model file
  std::string s;      
  s = XMLModelFile::getGraphics(xml)->getString("model");
        
  // Offset of center of gravity
  CRRCMath::Vector3  pCG;         
  pCG = CRRCMath::Vector3(0, 0, 0);
  if (xml->indexOfChild("CG") >= 0)
  {
    SimpleXMLTransfer* i;
    i = xml->getChild("CG");
    pCG.r[0] = i->attributeAsDouble("x", 0);
    pCG.r[1] = i->attributeAsDouble("y", 0);
    pCG.r[2] = i->attributeAsDouble("z", 0);
    
    if (i->attributeAsInt("units") == 1)
      pCG *= M_TO_FT;
  }
  
  // plib automatically loads the texture file, but it does not know which directory to use.
  // where is the object file?
  std::string of = FileSysTools::getDataPath("objects/" + s);
  // compile and set relative texture path
  std::string tp = of.substr(0, of.length()-s.length()-1-7) + "textures";    

  lVisID = Video::new_visualization(of, tp, pCG, xml);
  
  if (lVisID == INVALID_AIRPLANE_VISUALIZATION)
  {
    std::string msg = "Unable to open airplane model file \"";
    msg += s;
    msg += "\"\nspecified in \"";
    msg += xml->getSourceDescr();
    msg += "\"";
    throw std::runtime_error(msg);
  }

  // get half (approxiamtely) of the airplane size from the graphical model
  airplaneRadius = Video::get_radius(lVisID);
}


CRRCAirplaneV2::~CRRCAirplaneV2()
{
  if (lVisID != INVALID_AIRPLANE_VISUALIZATION)
  {
    Video::delete_visualization(lVisID);
  }
}


void CRRCAirplaneV2::initSound(SimpleXMLTransfer* xml)
{
  SimpleXMLTransfer* cfg = XMLModelFile::getConfig(xml);
  SimpleXMLTransfer* sndcfg = cfg->getChild("sound", true);
  int children = sndcfg->getChildCount();
  int units = sndcfg->getInt("units", 0);
  
  for (int i = 0; i < children; i++)
  {
    SimpleXMLTransfer *child = sndcfg->getChildAt(i);
    std::string name = child->getName();
    
    if (name.compare("sample") == 0)
    {
      T_AirplaneSound *sample;

      // assemble relative path
      std::string soundfile;
      soundfile           = child->attribute("filename");

      // other sound attributes
      int sound_type      = child->getInt("type", SOUND_TYPE_GLIDER);
      double dPitchFactor = child->getDouble("pitchfactor", 0.002);
      double dMaxVolume   = child->getDouble("maxvolume", 1.0);
  
      if (dMaxVolume < 0.0)
      {
        dMaxVolume = 0.0;
      }
      else if (dMaxVolume > 1.0)
      {
        dMaxVolume = 1.0;
      }

  //~ if (cfg->indexOfChild("power") < 0)
    //~ max_thrust = 0;
  //~ else
    //~ max_thrust = 1;
  
      if (soundfile != "")
      {
        // Get full path (considering search paths). 
        soundfile = FileSysTools::getDataPath("sounds/" + soundfile);
      }
      
      // File ok? Use default otherwise.
      if (!FileSysTools::fileExists(soundfile))
        soundfile = FileSysTools::getDataPath("sounds/fan.wav");
    
      std::cout << "soundfile: " << soundfile << "\n";
      //~ std::cout << "max_thrust: " << max_thrust << "\n";
      std::cout << "soundserver: " << Global::soundserver << "\n";
  
      // Only make noise if a sound file is available
      if (soundfile != "" && Global::soundserver != (CRRCAudioServer*)0)
      {        
        std::cout << "Using airplane sound " << soundfile << ", type " << sound_type << ", max vol " << dMaxVolume << std::endl;
        
        if (sound_type == SOUND_TYPE_GLIDER)
        {
          T_GliderSound *glidersound;
          float flMinRelV, flMaxRelV, flMaxDist;
          flMinRelV = (float)child->getDouble("v_min", 1.5);
          flMaxRelV = (float)child->getDouble("v_max", 4.0);
          flMaxDist = (float)child->getDouble("dist_max", 300);
          
          if (units == 1)
          {
            // convert from metric units to ft.
            flMaxDist *= M_TO_FT;
          }
          
          glidersound = new T_GliderSound(soundfile.c_str(), Global::soundserver->getAudioSpec());
          glidersound->setMinRelVelocity(flMinRelV);
          glidersound->setMaxRelVelocity(flMaxRelV);
          glidersound->setMaxDistanceFeet(flMaxDist);
          sample = glidersound;
        }
        else
        {
          sample = new T_EngineSound(soundfile.c_str(), Global::soundserver->getAudioSpec());
        }
                
        sample->setType(sound_type);
        sample->setPitchFactor(dPitchFactor);
        sample->setMaxVolume(dMaxVolume);
        sample->setChannel(Global::soundserver->playSample((T_SoundSample*)sample));
        sound.push_back(sample);
      }
    }
  }
}


/*****************************************************************************/

/** \brief Update the airplane's  visualisation
 *
 *  This method does not draw anything. It only
 *  updates the aircraft's visualization with the
 *  aircraft's current position and orientation.
 *  The actual drawing handled internally by mod_video.
 *
 *  \param airplane pointer to the airplane's FDM object
 */
void CRRCAirplaneV2::update(FDMBase* airplane)
{
  Video::set_position(lVisID,
                      airplane->getPos(),
                      airplane->getPhi(),
                      airplane->getTheta(),
                      airplane->getPsi());
}


/** \brief Draw the airplane's visualisation
 *
 *  It only draws the airplane object, without casting shadows.
 *  Shadow casting is currently only handled by function
 *  Video::draw_airplanes() which draws all defined airplanes
 *  and their casted shadows.
 *
 */
void CRRCAirplaneV2::draw()
{
  Video::draw_airplane(lVisID, Global::Simulation->getSimulationTimeSinceReset());
}


/** \brief Reset the airplane's visualisation
 *
 *  It currently only reset airplane's shadow
 *
 *  \param airplane pointer to the airplane's FDM object
 */
void CRRCAirplaneV2::reset(FDMBase* airplane)
{
  Video::reset_shadow(lVisID);
}
