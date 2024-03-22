/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2008-2009 Jan Reucker (original author)
 * Copyright (C) 2009-2010 Jens W. Wulf
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
  

/** \file aircraft.cpp
 *
 * A compound class that collects everything that exists "per airplane".
 */

#include "aircraft.h"
#include "mod_fdm/formats/airtoxml.h"
#include "mod_fdm/xmlmodelfile.h"
#include "mod_robots/robot.h"
#include "mod_misc/filesystools.h"


/**
 * Create an Aircraft
 */
Aircraft::Aircraft()
: model_(NULL), fdmInterface(new ModFDMInterface()), fdmInterfaceBackup(NULL), latest_configfile(NULL)
{
}


/**
 * Destroy an Aircraft
 */
Aircraft::~Aircraft()
{
  cleanup();
}


void Aircraft::cleanup()
{
  if (fdmInterface != 0)
    delete fdmInterface;
  if (model_ != 0)
    delete model_;
  delete fdmInterfaceBackup;
  fdmInterface = NULL;
  fdmInterfaceBackup = NULL;
  model_ = NULL;
}


/**
 * Get the Aircraft's positon in world coordinates.
 */
CRRCMath::Vector3 Aircraft::getPos()
{
  if (fdmInterface != NULL)
  {
    return fdmInterface->fdm->getPos();
  }
  else
  {
    return CRRCMath::Vector3(0.0, 0.0, 0.0);
  }
}


/**
 * Set the Aircraft's FDM interface
 */
void Aircraft::setFDMInterface(ModFDMInterface *fdm)
{
  fdmInterface = fdm;
}


/**
 * Get a pointer to the Aircraft's FDM
 */
FDMBase* Aircraft::getFDM() const
{
  if (fdmInterface != NULL)
  {
    return fdmInterface->fdm;
  }
  else
  {
    return NULL;
  }
}


/**
 * Set the CRRCAirplane instance to use as visual model
 */
void Aircraft::setModel(CRRCAirplane* model)
{
  if (model_ != NULL)
  {
    delete model_;
  }
  model_ = model;
}


/**
 * Enter testmode
 */
void Aircraft::enterTestmode(CRRCMath::Vector3 planeposn)
{
  if (fdmInterfaceBackup == NULL)
  {
    // backup the current FDM
    fdmInterfaceBackup = fdmInterface;

    // create a new FDM for testmode
    setFDMInterface(new ModFDMInterface());
    getFDMInterface()->loadAirplaneTestmode(-planeposn.r[2],
                                             planeposn.r[0],
                                            -planeposn.r[1]);
  }
}


/**
 * Leave testmode
 */
void Aircraft::leaveTestmode()
{
  if (fdmInterfaceBackup != NULL)
  {
    // delete testmode FDM
    delete fdmInterface;

    // restore previous FDM
    fdmInterface = fdmInterfaceBackup;
    fdmInterfaceBackup = NULL;
  }
}


int Aircraft::loadDemo(std::string demofilename)
{
  cleanup();
  
  ModRobotInterface* robot = new ModRobotInterface();
  fdmInterface = robot;
  
  try
  {
    robot->loadAirplane(demofilename.c_str(), (FDMEnviroment*)0, (SimpleXMLTransfer*)0);
    
    if (robot->robot)
    {
      SimpleXMLTransfer* header = robot->robot->GetHeader();
      std::string filename = FileSysTools::getDataPath(header->getString("airplane.file"));
    
      SimpleXMLTransfer* xml = new SimpleXMLTransfer(filename);

      // Here we copy graphics and config preferences from the demo file
      // into the in-memory-copy of the airplane. This is because an airplane file
      // should not be altered by user preferences.
      XMLModelFile::SetGraphics(xml, header->getInt("airplane.graphics"));
      XMLModelFile::SetConfig  (xml, 0);
      
      model_ = new CRRCAirplaneV2(xml);
      
      delete xml;
    }
  }
  catch (XMLException e)
  {
    std::string msg = "Error opening demo file: ";
    msg += demofilename;
    msg += ": ";
    msg += e.what();

    throw std::runtime_error(msg);    
  }  
  if (getFDM() == NULL)
  {
    throw std::runtime_error("Unable to load airplane demo file.");
  }
  
  return(0);
}

int Aircraft::load(SimpleXMLTransfer *configfile, FDMEnviroment* fdmEnvironment,
                   bool fReloadOnly)
{
  int nRetCode = 1;
  
  if (!fReloadOnly)
  {
    latest_configfile = configfile;
    cleanup();
    fdmInterface = new ModFDMInterface();
  }

  std::string filename = configfile->getString("airplane.file", "models/allegro.xml");
  filename = air_to_xml_file_load(filename);

  try
  {
    SimpleXMLTransfer* xml = new SimpleXMLTransfer(filename);

    SimpleXMLTransfer* ap = configfile->getChild("airplane");

    // Here we copy graphics and config preferences from crrcsim's config file
    // into the in-memory-copy of the airplane. This is because an airplane file
    // should not be altered by user preferences.
    XMLModelFile::SetGraphics(xml, ap->attributeAsInt("graphics", 0));
    XMLModelFile::SetConfig  (xml, ap->attributeAsInt("config",   0));

    if (fReloadOnly)
      nRetCode = fdmInterface->ReloadParams(xml, configfile);
    else
    {
      fdmInterface->loadAirplane(xml, fdmEnvironment, configfile);
      if (configfile->getInt("video.enabled", 1))
      {
        model_ = new CRRCAirplaneV2(xml);
      }
    }

    delete xml;
  }
  catch (XMLException e)
  {
    std::string msg = "Error opening airplane specification file: ";
    msg += filename;
    msg += ": ";
    msg += e.what();

    throw std::runtime_error(msg);    
  }  
  if (getFDM() == NULL)
  {
    throw std::runtime_error("Unable to load airplane specification file.");
  }
  
  return(nRetCode);
}

int Aircraft::ReloadParams()
{
  if (latest_configfile)
  {
    int nRetCode;
    try
    {
      nRetCode = load(latest_configfile, 0, true);
    }
    catch (std::runtime_error& e)
    {
      return(-1);
    }
    return(nRetCode);
  }
  else
    return(0);
}
