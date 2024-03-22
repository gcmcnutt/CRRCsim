/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2006, 2008 - Jens Wilhelm Wulf (original author)
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
#include "xmlmodelfile.h"

#include <string>
#include <iostream>

namespace XMLModelFile
{

  bool ListOptions(SimpleXMLTransfer* xml)
  {
    SimpleXMLTransfer* it;
    SimpleXMLTransfer* e;
    SimpleXMLTransfer* grp;
    SimpleXMLTransfer* cfg;
    SimpleXMLTransfer* opt;
    unsigned int       uSize;
    
    // insert "options" group
    opt = new SimpleXMLTransfer();
    opt->setName("options");
    xml->addChild(opt);
    // group for graphics options and default value
    grp = new SimpleXMLTransfer();
    grp->setName("graphics");
    grp->setAttribute("opt", "0");
    opt->addChild(grp);
    // group for config options and default value
    cfg = new SimpleXMLTransfer();
    cfg->setName("config");
    cfg->setAttribute("opt", "0");
    opt->addChild(cfg);
    
    // Copy options and descriptions to the new group
    uSize = xml->getChildCount();
    for (unsigned int uCnt=0; uCnt<uSize; uCnt++)
    {
      it = xml->getChildAt(uCnt);

      if (it->getName().compare("graphics") == 0)
      {
        e = new SimpleXMLTransfer();
        e->setName("graphics");
        // todo: maybe use other languages (if available and configured?)
        e->setContent(it->getChild("descr_short.en")->getContentString());
        grp->addChild(e);
      }
      else if (it->getName().compare("config") == 0)
      {
        e = new SimpleXMLTransfer();
        e->setName("config");
        // todo: maybe use other languages (if available and configured?)
        e->setContent(it->getChild("descr_short.en")->getContentString());
        cfg->addChild(e);
      }
    }
    // debug: show options
    //std::cout << "Options:\n";
    //opt->print(std::cout, 5);

    return(cfg->getChildCount() > 1 || grp->getChildCount() > 1);
  }

  SimpleXMLTransfer* getGraphics(SimpleXMLTransfer* xml)
  {
    // which one to use?
    int                nNr   = xml->getInt("options.graphics.opt");
    unsigned int       uSize = xml->getChildCount();
    SimpleXMLTransfer* it;
    SimpleXMLTransfer* first = (SimpleXMLTransfer*)0;
    int                nGCnt = -1;

    for (unsigned int uCnt=0; uCnt<uSize; uCnt++)
    {
      it = xml->getChildAt(uCnt);

      if (it->getName().compare("graphics") == 0)
      {
        if (first == (SimpleXMLTransfer*)0)
          first = it;
        
        nGCnt++;
        if (nGCnt == nNr)
          return(it);
      }
    }
    
    return(first);
  }

  SimpleXMLTransfer* getConfig(SimpleXMLTransfer* xml)
  {
    // which one to use?
    int                nNr   = xml->getInt("options.config.opt");
    unsigned int       uSize = xml->getChildCount();
    SimpleXMLTransfer* it;
    SimpleXMLTransfer* first = (SimpleXMLTransfer*)0;
    int                nGCnt = -1;

    for (unsigned int uCnt=0; uCnt<uSize; uCnt++)
    {
      it = xml->getChildAt(uCnt);

      if (it->getName().compare("config") == 0)
      {
        if (first == (SimpleXMLTransfer*)0)
          first = it;
        
        nGCnt++;
        if (nGCnt == nNr)
          return(it);
      }
    }
    return(first);
  }

  SimpleXMLTransfer* getLaunchPresets(SimpleXMLTransfer* xml)
  {
    SimpleXMLTransfer* launch_presets = 0;
    
    try
    {
      SimpleXMLTransfer *launch = xml->getChild("launch");
      // the file had a <launch> tag, copy it
      launch_presets = new SimpleXMLTransfer(launch);
      // just make sure that we have at least one child,
      // else fall back to NULL
      if (launch_presets->getChildCount() <= 0)
      {
        delete launch_presets;
        launch_presets = NULL;
      }
    }
    catch (XMLException e)
    {
      launch_presets = NULL;
    }
    
    return(launch_presets);
  }

  SimpleXMLTransfer* getMixerPresets(SimpleXMLTransfer* xml)
  {
    SimpleXMLTransfer* mixer_presets = 0;
    
    try
    {
      SimpleXMLTransfer *mixer = xml->getChild("mixer");
      // the file had a <mixer> tag, copy it
      mixer_presets = new SimpleXMLTransfer(mixer);
      // just make sure that we have at least one child,
      // else fall back to NULL
      if (mixer_presets->getChildCount() <= 0)
      {
        delete mixer_presets;
        mixer_presets = NULL;
      }
    }
    catch (XMLException e)
    {
      mixer_presets = NULL;
    }
    
    return(mixer_presets);
  }

  void SetGraphics(SimpleXMLTransfer* xml, int nIdx)
  {
    xml->setAttributeOverwrite("options.graphics.opt", nIdx);
  }

  void SetConfig(SimpleXMLTransfer* xml, int nIdx)
  {
    xml->setAttributeOverwrite("options.config.opt", nIdx);
  }

  TSimInputs::eSteeringMap GetSteering(std::string smstr)
  {
    if (smstr == "AILERON")
      return(TSimInputs::smAILERON);
    else if (smstr == "ELEVATOR")
      return(TSimInputs::smELEVATOR);
    else if (smstr == "RUDDER")
      return(TSimInputs::smRUDDER);
    else if (smstr == "THROTTLE")
      return(TSimInputs::smTHROTTLE);
    else if (smstr == "FLAP")
      return(TSimInputs::smFLAP);
    else if (smstr == "SPOILER")
      return(TSimInputs::smSPOILER);
    else if (smstr == "RETRACT")
      return(TSimInputs::smRETRACT);
    else if (smstr == "PITCH")
      return(TSimInputs::smPITCH);
    else if (smstr == "NOTHING")
      return(TSimInputs::smNOTHING);
    else
    {
      std::cerr << "fdm: unknown steering mapping attribute: \"" << smstr << "\"" << std::endl;
      std::cerr << "fdm: falling back to NOTHING" << std::endl;
      return(TSimInputs::smNOTHING);
    }    
  }
   
};
