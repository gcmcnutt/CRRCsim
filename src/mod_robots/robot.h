/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2010 - Jens Wilhelm Wulf (original author)
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
#ifndef CRRC_ROBOT_H
# define CRRC_ROBOT_H

# include <string>
# include <iostream>
# include <fstream>

# include "../mod_fdm/fdm.h"

/**
 * This class is the base for all airplanes which are not controlled by 
 * the local user, but from another user via network, a robot or a flight
 * log file. 
 * It is derived from the FDM class (used by the local users airplane) in order
 * to provide demo mode (controlling the local user's airplane).
 * 
 * @author Jens Wilhelm Wulf
 */
class RobotBase : public FDMBase
{
public:
  
  /**
   * The world coordinate vector vWorld is transformed
   * to body coordinates and returned.
   */
  virtual CRRCMath::Vector3 WorldToBody(CRRCMath::Vector3 vWorld) { return(CRRCMath::Vector3()); };
  
  virtual CRRCMath::Vector3 getPos() { return(v3Pos); };
  
  virtual double getPhi()      { return(v3Euler.r[0]); };
  virtual double getTheta()    { return(v3Euler.r[1]); };
  virtual double getPsi()      { return(v3Euler.r[2]); };
  virtual bool   isStalling()  { return(false); };
  virtual double getFlightCL() { return(0.0); };
  
  /**
   * returns velocity w.r.t. earth surface
   */
  virtual CRRCMath::Vector3 getVel()   { return(CRRCMath::Vector3()); };
  virtual CRRCMath::Vector3 getAccel() { return(CRRCMath::Vector3()); };
  
  virtual CRRCMath::Vector3 getPQR()   { return(CRRCMath::Vector3()); };
  
  virtual double getLon() { return(-1.0); };
  virtual double getLat() { return(-1.0); };
  virtual double getAlt() { return(-1.0); };
  
  RobotBase();
  
  virtual ~RobotBase() {};
  
  /**
   * Used for sound calculation. It returns the prop's number of revolutions
   * per second [1/s].
   */
  virtual double getPropFreq() { return(0); };
  
  /**
   * Returns velocity relative to airmass [ft/s].
   */
  virtual double getVRelAirmass() { return(0); };
  
  /**
   * Returns relative battery capacity/fuel left (0..1).
   */
  virtual double getBatCapLeft() { return(0.68); };
  
  /**
   * Wingspan of the aircraft in feet
   */
  virtual double getWingspan()  { return(1); };
  
  /**
   * computed velocity for trimmed flight in dead air
   */
  virtual double getTrimmedFlightVelocity()  { return(1); };
  
  /**
   * returns Z coordinate of lowest point
   */
  virtual double getZLow()  { return(0); };
  
  /**
   * This does only reload airplane parameters, but does not change states.
   * An FDM doesn't need to implement this method.
   * 
   * returns 0 if nothing happened, 1 on success, throws an XMLException otherwise.
   */
  virtual int ReloadParams(SimpleXMLTransfer* xml,
                           SimpleXMLTransfer* cfg) { return(0); };
  
  FDMEnviroment* GetEnv() { return((FDMEnviroment*)0); };
  
  /**
   * Return the file's XML header. This has been introduced for FDMs 
   * which aren't really FDMs (like playback and network source)
   */
  virtual SimpleXMLTransfer* GetHeader() { return(header); };
  
  /**
   *
   */
  virtual void ReceiveMarker(int id) {};
  
protected:
  SimpleXMLTransfer* header;
  CRRCMath::Vector3 v3Pos;
  CRRCMath::Vector3 v3Euler;
};

/**
 * This relates to ModFDMInterface just like RobotBase relates to FDMBase. 
 * 
 * @author Jens Wilhelm Wulf
 */
class ModRobotInterface : public ModFDMInterface
{
public:
  
  ModRobotInterface();
  
  ~ModRobotInterface();
    
  virtual void Clean();
  
  /**
   * Load robot description from file. This method should be able to handle
   * different filetypes (create an instance of the correct RobotBase).
   */
  virtual void loadAirplane(const char* filename, 
                            FDMEnviroment* myEnv,
                            SimpleXMLTransfer* cfg);
  
  /**
   * Pointer to FDM in use.
   */
  RobotBase* robot;

};

#endif
