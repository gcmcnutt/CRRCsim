/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2004-2006, 2008-2009 Jens Wilhelm Wulf (original author)
 * Copyright (C) 2005, 2006 Jan Reucker
 * Copyright (C) 2006 Todd Templeton
 * Copyright (C) 2008 Joel Lienard
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
  

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mod_misc/SimpleXMLTransfer.h"

#include <string>
#include <vector>

class T_Config;

extern SimpleXMLTransfer* cfgfile;
extern T_Config*          cfg;


/**
 * Save options back to crrcsim.xml
 */
int options_saveToFile();

/**
 * This function should save anything back to SimpleXMLTransfer* cfgfile
 * which isn't already there!
 */
int options_saveToConfig();


/**
 * Returns <code>true</code> if the current settings are not equal
 * to those in the config file.
 */
bool options_changed();

/**
 * Default model start position relative to player. Ideally this position is
 * a mix of what was used on 'Cape Cod' and 'Davis', but this would lead to
 * a very low start (near the water) at 'Cape Cod', so this is different.
 */
#define MODELSTART_REL_FRONT  4
#define MODELSTART_REL_RIGHT  5


/**
 * This file includes settings and flags for flight location, thermal and
 * wind characteristics, dynamic soaring. Everything regarding these
 * values (changing, reading from config, writing to config) has to be 
 * done using T_Config and its public interface.
 * 
 * As long as this is done, everything should be ok.
 * 
 * As of today, these efforts were made to ensure settings
 * for different locations are always loaded and saved correctly,
 * and the setup for dynamic soaring is valid.
 * 
 * This class delivers information about where files (configuration, data) can
 * be found.
 * 
 * Jens Wilhelm Wulf, 10.01.2005
 */
class T_Config;

/**
 * Thermal characteristics
 */
class T_Thermal
{
  public:
   T_Thermal();
      
   /**
    * Read settings from <code>cfgfile</code>, depending on data
    * from <code>cfg</code>.
    */
   void read(SimpleXMLTransfer* cfgfile,
             T_Config*          cfg);
   
   /**
    * writes configuration back into the configuration cfgfile
    */
   int putBackIntoCfg(SimpleXMLTransfer* cfgfile);
  
  
  // The following datafields are only a buffer to prevent reading from XML too often!
   float strength_mean;      // Mean strength of thermal in ft/s
   float strength_sigma;     // 1 sigma variation in thermal strength
   float radius_mean;        // Mean thermal radius in feet
   float radius_sigma;       // 1 sigma thermal radius variation
   float density;            // How many thermals per square foot -- if
                             // the unit of every length here is foot, as it seems.
   float lifetime_mean;      // Average lifetime of a thermal in seconds
   float lifetime_sigma;     // 1 sigma variation in lifetime in seconds      
};

/**
 * Wind characteristics.
 */
class T_Wind
{
  public:
   T_Wind();
   
   void setDefaults();
   
   /**
    * Read settings from <code>cfgfile</code>, depending on data
    * from <code>cfg</code>.
    */
   void read(SimpleXMLTransfer* cfgfile,
             T_Config*          cfg);

   /**
    * writes configuration back into the configuration cfgfile
    */
   int putBackIntoCfg(SimpleXMLTransfer* cfgfile);

   void setVelocity (double val);
   
   /**
    * One has to check whether dynamic soaring is active after changing the 
    * direction or flying location. Therefore this function needs a pointer
    * to the T_Config managing everything. This is done so one is not able
    * to call this function without making the check...
    */
   void setDirection(double val, T_Config* cfg);

   void setTurbulence(double val);
   
   float getVelocity () { return(velocity);  };
   
   float getDirection() { return(direction); };
   
   float getTurbulence() { return(turbulence); };
   
  private:
   
   /**
    * Wind in ft/sec
    */
   float velocity;
   
   /**
    * Direction of air molecule travel: where do they come from?
    * 0/360: from north
    * 90   : from east
    * 180  : from south
    * ...
    */
   float direction;
   
   /**
    * Wind turblence relative intenisty
    * 0  : no wind turbulence
    * <1 : reduced turbulence intensity
    * 1  : same as digital filter Dryden model from MIL-HDBK-1797
    * >1 : increased turbulence intensity
    */
   float turbulence;
};


/**
 * This class collects information about things the user configured, like
 * weather conditions, chosen plane, how to start the plane, place to fly at...
 */
class T_Config
{
  public:
   T_Thermal*    thermal;
   T_Wind*       wind;
     
   /**
    * path of configuration file crrcsim.xml
    */
   static void putConfigFilePath(std::string _ConfigFilePath);
   static std::string getConfigFilePath();
   
   
   /**
    * Set defaults, determine paths and load the configuration file
    * into <code>acfgfile</code>.
    */
   T_Config(SimpleXMLTransfer*& acfgfile);

   /**
    * determine path of config file and provide configuration in <code>acfgfile</code>
    */
   void provideConfigAndPath(SimpleXMLTransfer*& acfgfile);
   
   /**
    * Creates a default configuration in <code>acfgfile</code>. It should contain
    * values for every module which can't handle finding nothing in <code>acfgfile</code>.
    * It is called when there is no config file to load.
    */
   void createDefaultConfig(SimpleXMLTransfer*& acfgfile);
      
   /**
    * Read settings from <code>cfgfile</code>, call children to
    * do the same.
    */
   void read(SimpleXMLTransfer* cfgfile);
   
   /**
    * Make sure that every location based parameter for the current location
    * is available. Set defaults if this is not the case.
    * Calls children to do the same.
    */
   void checkLocationBasedPar(SimpleXMLTransfer* cfgfile);
   
   /**
    * Set location as string. Location based parameters
    * are read.
    */
   void setLocation(const char*         locstr,
                    SimpleXMLTransfer*  cfgfile);
   
   /**
    * Set location as string. Location based parameters
    * are read. Explicitely select sky variant.
    */
   void setLocation(const char*         locstr,
                    int                 sky_variant,
                    SimpleXMLTransfer*  cfgfile);
   
   /**
    * Get name of location
    */
   const char* getLocationName();
   
   /**
    * get scenery filename
    */
   const char* getSceneryFilename() const { return(scenery_filename.c_str()); };

   /**
    * get sky variant
    */
   int getSkyVariant() const { return nSkyVariant; };

   /**
    * Checks conditions for dynamic soaring
    */
   void checkDynamicSoaring();
   
   /**
    * Is dynamic soaring enabled?
    */
   int getDynamicSoaring() { return(dynamic_soaring); };
   
   /**
    * writes configuration back into the configuration cfgfile
    */
   int putBackIntoCfg(SimpleXMLTransfer* cfgfile);
   
   /**
    * Provides the SimpleXMLTransfer for the  location named 'location'
    */
   SimpleXMLTransfer* getLocCfgPtr(SimpleXMLTransfer* cfgfile,std::string location);
              
  /**
    * Provides the SimpleXMLTransfer for the current location
    */
   SimpleXMLTransfer* getCurLocCfgPtr(SimpleXMLTransfer* cfgfile);
              
   /**
    * Get paths of directories where .air-files may reside.
    * This usually is OS-dependent.
    */
   static void getModelDirs(std::vector<std::string>& dirlist);
   
   /**
    * Get paths of directories where scne descrition may reside.
     */
   static void getLocationDirs(std::vector<std::string>& dirlist);
   
   /**
    * Get paths of all directories that contain F3F sounds.
    * This usually is OS-dependent.
    */
   static void getF3FSoundDirs(std::vector<std::string>& dirlist);

  private:
   
   /**
    * path of sound file
    */
   static std::string soundfilePath;
   
   /**
    * path of configuration file
    */
   static std::string ConfigFilePath;
   
   /**
    * Set location as string. Location based parameters
    * are read if <code>fReadAgain</code>.
    */
   void setLocation(const char*         locstr,
                    SimpleXMLTransfer*  cfgfile,
                    bool                fReadAgain,
                    int                 sky_variant);
         
   /**
    * Which location to fly at?
    */
   int           nLocation;
   
   /**
    * Filename of scenery
    */
   std::string scenery_filename;

   /**
    * Which sky should be loaded from the scenery file?
    */
   int nSkyVariant;

   /**
    * 1 means dynamic soaring is active
    */
   int dynamic_soaring;
   
   
};
#endif
