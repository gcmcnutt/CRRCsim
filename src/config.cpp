/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2004-2006, 2008-2009 Jens Wilhelm Wulf (original author)
 * Copyright (C) 2005, 2006, 2007, 2008 Jan Reucker
 * Copyright (C) 2005 Lionel Cailler
 * Copyright (C) 2006 Todd Templeton
 * Copyright (C) 2008, 2009 Joel Lienard
 * Copyright (C) 2008 Olivier Bordes
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
  

#include <fstream>

#include "global.h"
#include "config.h"
#include "mod_misc/filesystools.h"
#include "mod_misc/lib_conversions.h"

#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <errno.h>
#include "defines.h"
#include "mod_misc/filesystools.h"
#include "zoom.h"
#include "crrc_main.h"
#include "mod_landscape/crrc_scenery.h"
#include "mod_windfield/windfield.h"

#ifdef linux
# include <stdlib.h>     // getenv()
#endif

#if defined(__APPLE__) || defined(MACOSX)
# include <stdlib.h>     // getenv()
#endif

/* ******************************************************************************* */

// Pointer to the new configuration file. This should be passed
// to every module/object which wants to read something from the
// config (crrcsim.xml).
SimpleXMLTransfer* cfgfile = (SimpleXMLTransfer*)0;

// 
T_Config*       cfg;

/* ******************************************************************************* */

// description: see header file
int options_saveToFile()
{
  options_saveToConfig();
  
  {
    std::ofstream outfile;
    std::string   path = T_Config::getConfigFilePath();

    if (path.find('/') != std::string::npos)
      FileSysTools::makeSurePathExists(path.substr(0, path.rfind('/')));

    outfile.open(path.c_str());
    if (!outfile)
      return(-1);
    
    cfgfile->print(outfile);
    
    outfile.close();
  }
  return(0);
}

// description: see header file
int options_saveToConfig()
{
  // Some options need to be put back into the SimpleXMLTransfer
  zoom_putBackIntoCfg();
  write_globals_into_config();
  cfg->putBackIntoCfg(cfgfile);
  Global::TXInterface->putBackIntoCfg(cfgfile);
  Global::inputDev->putBackIntoCfg(cfgfile);

  return(0);
}

// description: see header file
bool options_changed()
{
  bool fEqual;
  
  options_saveToConfig();
  
  // Read config file
  SimpleXMLTransfer* cfgfileold = (SimpleXMLTransfer*)0;
  cfg->provideConfigAndPath(cfgfileold);
  
  fEqual = cfgfile->equalsOrdered(cfgfileold);

  delete cfgfileold;
  
  if (fEqual)
  {
    printf("options are unchanged\n");
    return(false);
  }
  else
  {
    printf("options have been changed\n");
    // cfgfile->print(std::cout); // test
    return(true);
  }
}

/* ******************************************************************************* */

T_Thermal::T_Thermal()
{
}

void T_Thermal::read(SimpleXMLTransfer* cfgfile,
                     T_Config*          cfg)
{
  SimpleXMLTransfer* logcfgptr = cfg->getCurLocCfgPtr(cfgfile);
  
  if (logcfgptr->indexOfChild("thermal") < 0)
    logcfgptr->addChild(GetDefaultConf_Thermal());
  
  // Read buffer data
  SimpleXMLTransfer* el = logcfgptr->getChild("thermal");  
  strength_mean   = el->getDouble("strength_mean");
  strength_sigma  = el->getDouble("strength_sigma");
  radius_mean     = el->getDouble("radius_mean");
  radius_sigma    = el->getDouble("radius_sigma");
  density         = el->getDouble("density");
  lifetime_mean   = el->getDouble("lifetime_mean");
  lifetime_sigma  = el->getDouble("lifetime_sigma");

  // show values:
  printf("Thermals: strength_mean=%f strength_sigma=%f radius_mean=%f radius_sigma=%f\n",
         strength_mean, strength_sigma,
         radius_mean, radius_sigma);
  printf("Thermals: density=%f lifetime_mean=%f lifetime_sigma=%f\n",
         density, 
         lifetime_mean, lifetime_sigma);  
}

int T_Thermal::putBackIntoCfg(SimpleXMLTransfer* cfgfile)
{
  // This is not necessary anymore as there is no internal data.
  return(0);
}

/* ******************************************************************************* */

void T_Wind::setDefaults()
{
  velocity   = 7;
  direction  = 270;
  turbulence = 1; // turbulence on, standard intensity
}

void T_Wind::setVelocity(double val)
{
  velocity = val;  
}

void T_Wind::setDirection(double val, T_Config* cfg)
{
  // Sanity check, see description in header file
  if (cfg->wind == this)
  {  
    direction = val;
        
    cfg->checkDynamicSoaring();
  }
}

void T_Wind::setTurbulence(double val)
{
  turbulence = val;  
}

T_Wind::T_Wind()
{
  setDefaults();
}

/**
 *  Read the values for the current wind configuration from the
 *  main config file. If no entry exists for the current location,
 *  try to get a sensible default from the current location object
 *  itself. If currently no scenery is loaded, use a global default.
 *
 *  In any case, don't write anything back into the configuration or
 *  strange things might happen...
 */
void T_Wind::read(SimpleXMLTransfer* cfgfile, T_Config* cfg)
{
  // the global defaults
  float flDirection = 270;
  float flSpeed = 7;
  float wdTurbulence = 1;
  int ImposeDirection = false;
  
  // try to override defaults with values from the current scenery object
  if (Global::scenery != NULL)
  {
    flDirection = Global::scenery->getDefaultWindDirection();
    flSpeed = Global::scenery->getDefaultWindSpeed();
    ImposeDirection = Global::scenery->getImposeWindDirection();
    wdTurbulence = Global::scenery->getDefaultTurbulence();
    printf("Default wind from scenery: %.2f ft/s at %.2f deg, turbulence (rel.) %.1f\n",
           flSpeed, flDirection, wdTurbulence);
  }
  
  // now try to override these defaults with values from the config file
  try
  {
    SimpleXMLTransfer* el = cfg->getCurLocCfgPtr(cfgfile)->getChild("wind");
    if(!ImposeDirection) flDirection = el->getDouble("direction");
    flSpeed = el->getDouble("velocity");
    wdTurbulence = el->getDouble("turbulence");
  }
  catch (XMLException e)
  {
    // nothing in the config file...
  }
  setVelocity(flSpeed);
  setDirection(flDirection, cfg);
  setTurbulence(wdTurbulence);
    
  // show values:
  printf("Wind: %.2f ft/s at %.2f deg, turbulence (rel.) %.1f\n",
         velocity, direction, turbulence);
}

int T_Wind::putBackIntoCfg(SimpleXMLTransfer* cfgfile)
{
  SimpleXMLTransfer* el = cfg->getCurLocCfgPtr(cfgfile)->getChild("wind", true);

  el->setAttributeOverwrite("velocity", doubleToString(velocity));
  el->setAttributeOverwrite("direction", doubleToString(direction));
  el->setAttributeOverwrite("turbulence", doubleToString(turbulence));

  return(0);
}

/* ******************************************************************************* */

/** static members */
std::string T_Config::ConfigFilePath;
std::string T_Config::soundfilePath;

T_Config::T_Config(SimpleXMLTransfer*& acfgfile)
{
  // Defaults
  nLocation       = Scenery::DAVIS;
  thermal         = new T_Thermal();
  wind            = new T_Wind();
  dynamic_soaring = FALSE;

  provideConfigAndPath(acfgfile);
}

void T_Config::provideConfigAndPath(SimpleXMLTransfer*& acfgfile)
{
  // determine path and open configuration file: depends on operating system
  acfgfile = (SimpleXMLTransfer*)0;
  
  if (ConfigFilePath.length() <= 0)
  {
#ifdef linux
  // configuration file
  // If there is a crrcsim.xml in the working directory, take this one. If not,
  // ~/.crrcsim/crrcsim.xml is the one to choose.
  {
    ConfigFilePath = "crrcsim.xml";
    try
    {
      acfgfile = new SimpleXMLTransfer(ConfigFilePath);
    }
    catch (XMLException e)
    {
      char* homepath = getenv("HOME");
      if (homepath != (char*)0)
      {
        ConfigFilePath  = homepath;
        ConfigFilePath += "/.crrcsim/crrcsim.xml";        
      }
    }
  }
#elif defined(__APPLE__) || defined(MACOSX)
  // configuration file
  // If there is a crrcsim.xml in the working directory, take this one. If not,
  // ~/.crrcsim/crrcsim.xml is the one to choose.
  {
    ConfigFilePath = "crrcsim.xml";
    try
    {
      acfgfile = new SimpleXMLTransfer(ConfigFilePath);
    }
    catch (XMLException e)
    {
      char* homepath = getenv("HOME");
      if (homepath != (char*)0)
      {
        ConfigFilePath  = homepath;
        ConfigFilePath += "/Library/Preferences/crrcsim.xml";        
      }
    }
  }
#elif defined(WIN32)
  // configuration file
  // If there is a crrcsim.xml in the working directory, take this one. If not,
  // ~/.crrcsim/crrcsim.xml is the one to choose.
  {
    ConfigFilePath = "crrcsim.xml";
    try
    {
      acfgfile = new SimpleXMLTransfer(ConfigFilePath);
    }
    catch (XMLException e)
    {
      char* homepath = getenv("USERPROFILE");
      if (homepath != (char*)0)
      {
        ConfigFilePath  = homepath;
        ConfigFilePath += "/.crrcsim/crrcsim.xml";        
      }
    }
  }
# else
  ConfigFilePath = "crrcsim.xml";
# endif
  }
  
  printf("Configuration file is %s\n", ConfigFilePath.c_str());
  
  if (acfgfile == (SimpleXMLTransfer*)0)
  {
    try
    {
      acfgfile = new SimpleXMLTransfer(ConfigFilePath);
    }
    catch (XMLException e)
    {
      // So there is no configuration file to load. Create default values.
      createDefaultConfig(acfgfile);
    }
  }
  
  
  // Take a look at the version number of the config file. Maybe
  // it is an old, incompatible one and there is something we need to
  // do about it?
  {
    int nVer = acfgfile->attributeAsInt("version", 1);

    printf("Configuration version is %i\n", nVer);
        
    if (nVer < 2)
    {
      // Coordinate system has changed (initial airplane position, wind 
      // direction).
      // Things which need to be changed:
      //   -wind presets
      //   -initial airplane position for every location
      //   -wind direction for every location
      // This is not impossible, but I am too lazy now, so I just go on as if there had been no config file.
      delete acfgfile;      
      acfgfile = (SimpleXMLTransfer*)0;
      createDefaultConfig(acfgfile);      
    }
  }    
}

void T_Config::createDefaultConfig(SimpleXMLTransfer*& acfgfile)
{
  acfgfile = new SimpleXMLTransfer();
  acfgfile->setName("crrcsimConfig");
  
  // fill with necessary data
  acfgfile->makeSureAttributeExists("version", "2");
  acfgfile->makeSureAttributeExists("video.skybox.texture_offset", 
                                    ftoStr(DEFAULT_SKYBOX_TEXTURE_OFFSET, 1, 5).c_str());
  
  SimpleXMLTransfer* tex;

  // things which are not really necessary, but useful:
  //   -launch presets  
  tex = acfgfile->getChild("launch.preset", true);
  tex->makeSureAttributeExists("name_en",      "hand");
  tex->makeSureAttributeExists("altitude",     "10");
  tex->makeSureAttributeExists("velocity_rel", "1");
  tex->makeSureAttributeExists("angle",        "0");
  tex->makeSureAttributeExists("rel_to_player", "1");
  tex->makeSureAttributeExists("rel_front",     "0");
  tex->makeSureAttributeExists("rel_right",     "2");
  
  tex = new SimpleXMLTransfer();
  tex->setName("preset");
  tex->addAttribute("name_en",      "winch");
  tex->addAttribute("altitude",     "300");
  tex->addAttribute("velocity_rel", "1");
  tex->addAttribute("angle",        "0");
  tex->addAttribute("sal",          "0");
  acfgfile->getChild("launch")->addChild(tex);
  
  tex = new SimpleXMLTransfer();
  tex->setName("preset");
  tex->addAttribute("name_en",      "throw");
  tex->addAttribute("altitude",     "10");
  tex->addAttribute("velocity_rel", "2");
  tex->addAttribute("angle",        "0");
  tex->addAttribute("sal",          "0");
  tex->addAttribute("rel_to_player", "1");
  tex->addAttribute("rel_front",     "0");
  tex->addAttribute("rel_right",     "2");
  acfgfile->getChild("launch")->addChild(tex);
  
  tex = new SimpleXMLTransfer();
  tex->setName("preset");
  tex->addAttribute("name_en",      "hlg");
  tex->addAttribute("altitude",     "10");
  tex->addAttribute("velocity_rel", "5");
  tex->addAttribute("angle",        "0.38");
  tex->addAttribute("sal",          "0");
  tex->addAttribute("rel_to_player", "1");
  tex->addAttribute("rel_front",     "0");
  tex->addAttribute("rel_right",     "2");
  acfgfile->getChild("launch")->addChild(tex);
  
  tex = new SimpleXMLTransfer();
  tex->setName("preset");
  tex->addAttribute("name_en",      "SAL");
  tex->addAttribute("altitude",     "5");
  tex->addAttribute("velocity_rel", "7");
  tex->addAttribute("angle",        "0.20");
  tex->addAttribute("sal",          "1");
  tex->addAttribute("rel_to_player", "1");
  tex->addAttribute("rel_front",     "0");
  tex->addAttribute("rel_right",     "2");
  acfgfile->getChild("launch")->addChild(tex);
	
  tex = new SimpleXMLTransfer();
  tex->setName("preset");
  tex->addAttribute("name_en",      "Cape Cod F3F");
  tex->addAttribute("altitude",     "10");
  tex->addAttribute("velocity_rel", "1");
  tex->addAttribute("angle",        "0");
  tex->addAttribute("sal",          "0");
  tex->addAttribute("rel_to_player", "1");
  tex->addAttribute("rel_front",     "2.5");
  tex->addAttribute("rel_right",     "-20");
  acfgfile->getChild("launch")->addChild(tex);
	
  tex = new SimpleXMLTransfer();
  tex->setName("preset");
  tex->addAttribute("name_en",      "motor");
  tex->addAttribute("altitude",     "0");
  tex->addAttribute("velocity_rel", "0");
  tex->addAttribute("angle",        "0");
  tex->addAttribute("sal",          "0");
  tex->addAttribute("rel_to_player", "1");
  tex->addAttribute("rel_front",     "15");
  tex->addAttribute("rel_right",     "0");
  acfgfile->getChild("launch")->addChild(tex);
  
  
  //   -thermal presets
  tex = GetDefaultConf_Thermal();
  tex->addAttribute("name_en", "Default (v3)");
  acfgfile->getChild("presets.thermal", true)->addChild(tex);
  
  tex = new SimpleXMLTransfer();
  tex->setName("thermal");
  tex->addAttribute("name_en",      "F3F, heli (no thermals)");
  tex->addAttribute("strength_mean",      "0");
  tex->addAttribute("strength_sigma",     "0");
  tex->addAttribute("radius_mean",        "0");
  tex->addAttribute("radius_sigma",       "0");
  tex->addAttribute("lifetime_mean",      "0");
  tex->addAttribute("lifetime_sigma",     "0");
  tex->addAttribute("density",            "0");
  acfgfile->getChild("presets.thermal", true)->addChild(tex);
  
  //   -wind presets
  //~ acfgfile->makeSureAttributeExists("presets.wind.wind.name_en",   "Dynamic Soaring Cape Cod");
  //~ acfgfile->makeSureAttributeExists("presets.wind.wind.velocity",  "13");
  //~ acfgfile->makeSureAttributeExists("presets.wind.wind.direction", "90");
  
  tex = new SimpleXMLTransfer();
  tex->setName("wind");
  tex->addAttribute("name_en",      "Dynamic soaring Cape Cod");
  tex->addAttribute("velocity",     "13");
  tex->addAttribute("direction",    "90");
  tex->addAttribute("turbulence",    "1");
  acfgfile->getChild("presets.wind", true)->addChild(tex);
  
  tex = new SimpleXMLTransfer();
  tex->setName("wind");
  tex->addAttribute("name_en",      "Soaring slow Cape Cod");
  tex->addAttribute("velocity",     "10");
  tex->addAttribute("direction",    "270");
  tex->addAttribute("turbulence",   "1");
  acfgfile->getChild("presets.wind")->addChild(tex);
  
  tex = new SimpleXMLTransfer();
  tex->setName("wind");
  tex->addAttribute("name_en",      "Soaring ruff Cape Cod");
  tex->addAttribute("velocity",     "18");
  tex->addAttribute("direction",    "270");
  tex->addAttribute("turbulence",   "1");
  acfgfile->getChild("presets.wind")->addChild(tex);
  
  tex = new SimpleXMLTransfer();
  tex->setName("wind");
  tex->addAttribute("name_en",      "F3F competition at Cape Cod");
  tex->addAttribute("velocity",     "30");
  tex->addAttribute("direction",    "270");
  tex->addAttribute("turbulence",   "1");
  acfgfile->getChild("presets.wind")->addChild(tex);
  
  tex = new SimpleXMLTransfer();
  tex->setName("wind");
  tex->addAttribute("name_en",      "Heli (no wind)");
  tex->addAttribute("velocity",     "0");
  tex->addAttribute("direction",    "180");
  tex->addAttribute("turbulence",   "0");
  acfgfile->getChild("presets.wind")->addChild(tex);
}

void T_Config::read(SimpleXMLTransfer* cfgfile)
{
  cfgfile->makeSureAttributeExists("location.name", "scenery/davis-orig.xml");
  cfgfile->makeSureAttributeExists("location.sky", "0");
  setLocation(cfgfile->getString("location.name").c_str(), 
              cfgfile,
              false,
              cfgfile->getInt("location.sky"));

  checkLocationBasedPar(cfgfile);
  
  thermal->read(cfgfile, this);
  wind->read(cfgfile, this);
}

/// Checks for CAPE_COD and dynamic soaring conditions
void T_Config::checkDynamicSoaring()
{
  // if we have already loaded a valid scenery, update the
  // internal location ID
  if (Global::scenery != NULL)
  {
    nLocation = Global::scenery->getID();
  }
  
  if (nLocation == Scenery::CAPE_COD)
  {
    if ((wind->getDirection() < 360) && (wind->getDirection() > 180))
    {
      if (wind->getDirection() != 270)
      {
        printf("Off normal winds not supported yet.\n");
        // This results in me calling myself once again -- but
        // the next time this 'if' isn't true, so this is no problem.
        wind->setDirection(270, cfg);
      }
      printf("Sloping.\n");
      dynamic_soaring = FALSE;
    } 
    else 
    {
      if (wind->getDirection() != 90)
      {
        printf("Off normal winds not supported yet.\n");
        // This results in me calling myself once again -- but
        // the next time this 'if' isn't true, so this is no problem.
        wind->setDirection(90, cfg);
      }
      printf("Dynamic soaring.\n");
      dynamic_soaring = TRUE;
    }
  }      
  else
    dynamic_soaring = FALSE;
}

void T_Config::setLocation(const char*         locstr,
                           SimpleXMLTransfer*  cfgfile)
{
  setLocation(locstr, cfgfile, true, 0);
}

void T_Config::setLocation(const char*         locstr,
                           int                 sky_variant,
                           SimpleXMLTransfer*  cfgfile)
{
  setLocation(locstr, cfgfile, true, sky_variant);  
}

void T_Config::setLocation(const char*         locstr,
                           SimpleXMLTransfer*  cfgfile,
                           bool                fReadAgain,
                           int                 sky_variant)
{
  std::string tmp = trim(locstr);
  if (tmp.find("./") == 0)
    tmp = tmp.substr(2);

  printf("T_Config: Setting location to %s, sky variant %d\n", tmp.c_str(), sky_variant);
  scenery_filename  = tmp;
  nSkyVariant = sky_variant;

  // Set 
  cfgfile->setAttributeOverwrite("location.name", getLocationName());
  cfgfile->setAttributeOverwrite("location.sky", nSkyVariant);
 
  SimpleXMLTransfer* logcfgptr = cfg->getCurLocCfgPtr(cfgfile);
  logcfgptr->setAttributeOverwrite("sky.nUse",nSkyVariant);

  
  // reread config
  if (fReadAgain)
    read(cfgfile);
  
  // Location possibly changed, so this check is needed.
  // JR: this should be done from initialize_windfield()
  // checkDynamicSoaring();
}

const char* T_Config::getLocationName()
{
  return(scenery_filename.c_str());
}

int T_Config::putBackIntoCfg(SimpleXMLTransfer* cfgfile)
{
  thermal->putBackIntoCfg(cfgfile);
  wind->putBackIntoCfg(cfgfile);
  
  return(0);
}

SimpleXMLTransfer* T_Config::getCurLocCfgPtr(SimpleXMLTransfer* cfgfile)
{
  SimpleXMLTransfer* gr;
  // Now I can use the values specific to the chosen location
  std::string location = getLocationName();
  gr = getLocCfgPtr(cfgfile, location);
  return gr;
}

SimpleXMLTransfer* T_Config::getLocCfgPtr(SimpleXMLTransfer* cfgfile, std::string location)
{
  SimpleXMLTransfer* pgr;
  SimpleXMLTransfer* gr;
  
  try
  {
    pgr = cfgfile->getChild("locations", true);
    int size = pgr->getChildCount();
    for (int m=0; m<size; m++)
    {
      gr = pgr->getChildAt(m);
      if (gr->getName().compare("location") == 0 && gr->attribute("name").compare(location) == 0)
        return(gr);
    }
    // So there is no entry for the current location. Let's create one:
    gr = new SimpleXMLTransfer();
    gr->setName("location");
    gr->addAttribute("name", location);
    pgr->addChild(gr);
    return(gr);
  }
  catch (XMLException e)
  {
  }
  
  return((SimpleXMLTransfer*)0);
}

void T_Config::checkLocationBasedPar(SimpleXMLTransfer* cfgfile)
{ 
  //SimpleXMLTransfer* ptr;

  // make sure location entry is there
  //ptr = 
  getCurLocCfgPtr(cfgfile);

  // also check values read elsewhere
  /// \todo these defaults should already come from the scenery file!?!
  //no  if  every launch param position is relative/player position
  /*
  if (nLocation == Scenery::CAPE_COD)    
  {
    ptr->makeSureAttributeExists("position.airplane.x",  "-42");
    ptr->makeSureAttributeExists("position.airplane.y",  "10.5");
  }
  else
  {
    ptr->makeSureAttributeExists("position.airplane.x",  "-20.93");
    ptr->makeSureAttributeExists("position.airplane.y",  "0");
  }
  */
}

void T_Config::putConfigFilePath(std::string _ConfigFilePath)
{
  ConfigFilePath = _ConfigFilePath;
}


std::string T_Config::getConfigFilePath()
{
  return(ConfigFilePath);
}

void T_Config::getModelDirs(std::vector<std::string>& dirlist)
{
  FileSysTools::getSearchPathList(dirlist, "models");
}

void T_Config::getLocationDirs(std::vector<std::string>& dirlist)
{
  FileSysTools::getSearchPathList(dirlist, "scenery");
}


/**
 *  Create a list of F3F sound directories
 *
 *  F3F sound directories are all directories that reside in "sounds/f3f"
 *  somewhere in the search path and contain .wav files.
 *
 *  \param dirlist  This list of strings will be set to the retrieved paths.
 */
void T_Config::getF3FSoundDirs(std::vector<std::string>& dirlist)
{
  std::vector<std::string> paths;
  FileSysTools::getSearchPathList(paths, "sounds/f3f");

  // search the list of paths for subdirectories possibly containing
  // F3F wav files
  for (std::vector<std::string>::size_type i = 0; i < paths.size(); i++)
  {
    DIR *dir;
    if ((dir = opendir(paths[i].c_str())) == NULL)
    {
      std::string s = "T_Config::getF3FSoundDirs(): unable to open ";
      s += paths[i];
      perror(s.c_str());
    }
    else
    {
      struct dirent *ent;
      while ((ent = readdir(dir)) != NULL)
      {
        // test if this entity is really a directory
        std::string entity_fullpath = paths[i];
        entity_fullpath += "/";
        entity_fullpath += ent->d_name;
        
        DIR *subdir = opendir(entity_fullpath.c_str());
        if (subdir != NULL)
        {
          // Yes, this is a directory. We may immediately close it again
          // and see if we can add it to the list.
          closedir(subdir);
          
          // exclude some special directories
          if  ( (strcmp(ent->d_name, ".") != 0)       // exclude current dir
                  && 
                (strcmp(ent->d_name, "..") != 0)      // exclude parent dir
                  && 
                (strcmp(ent->d_name, "CVS") != 0)     // exclude CVS dir (mostly useless)
              )
          {
            // We've found a sound directory! Add it to the list.
            /// \todo Maybe we should check if this directory contains at least
            ///       one .wav file?
            dirlist.push_back(entity_fullpath);
          }
        }
        else
        {
          std::string s = "T_Config::getF3FSoundDirs(): ";
          s += entity_fullpath;
          perror(s.c_str());
        }
      }
      closedir(dir);
    }
  }
  
  std::cout << "F3F sound directories:" << std::endl;
  for (std::vector<std::string>::size_type i = 0; i < dirlist.size(); i++)
  {
    std::cout << "  " << dirlist[i] << std::endl;
  }
}

