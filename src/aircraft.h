/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2008 Jan Reucker (original author)
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
  

/** \file aircraft.h
 *
 * A compound class that collects everything that exists "per airplane".
 */

#include "mod_math/vector3.h"
#include "mod_fdm/fdm.h"
#include "crrc_loadair.h"

class Aircraft
{
  public:
    Aircraft();
    ~Aircraft();
    
    CRRCMath::Vector3 getPos();
    
    ModFDMInterface*  getFDMInterface() const {return fdmInterface;}
    void              setFDMInterface(ModFDMInterface *fdm);

    FDMBase*  getFDM() const;

    CRRCAirplane*     getModel() const {return model_;}
    void              setModel(CRRCAirplane* model);

    void  enterTestmode(CRRCMath::Vector3 planeposn);
    void  leaveTestmode();

    /**
     * (Re-)Load the airplane specified in configfile. Throw a
     * std::runtime_error on failure.
     * 
     * if fReloadOnly is set: only try to reload the FDM's parameters, but don't change any state.
     * 
     * returns 0 if nothing happened, 1 on success
     */
    int  load(SimpleXMLTransfer *configfile, FDMEnviroment* fdmEnvironment,
              bool fReloadOnly = false);
  
    /**
     * load demo/robot file
     */
    int loadDemo(std::string demofilename);
  
  
    /**
     * Tries to reload parameters without resetting any state or 
     * disturbing the simulation
     * 
     * returns 0 if nothing happened, 1 on success and -1 on error.
     */
    int ReloadParams();
  
    /**
     * Returns the most recently used config file
     */
    SimpleXMLTransfer* GetLatestConfig() { return(latest_configfile); };
  
  private:
    CRRCAirplane*      model_;               ///< The airplane model.
    ModFDMInterface*   fdmInterface;         ///< The fdm which is in use.
    ModFDMInterface*   fdmInterfaceBackup;   ///< Backup pointer when in test mode.
    SimpleXMLTransfer* latest_configfile;    ///< most currently used configfile
  
    void cleanup();
  
  
};

