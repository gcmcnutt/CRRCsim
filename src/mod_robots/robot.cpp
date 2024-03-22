/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2006, 2008-2010 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2006, 2007 - Jan Reucker
 *   Copyright (C) 2006 - Todd Templeton
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
#include "robot.h"

#include <iostream>

#include "fdm_playback.h"
#include "../mod_fdm/xmlmodelfile.h"

ModRobotInterface::ModRobotInterface()
{
  launch_presets = 0;
  robot = (RobotBase*)0;
}

ModRobotInterface::~ModRobotInterface()
{
  Clean();
}

void ModRobotInterface::Clean()
{
  if (robot != (RobotBase*)0)
    delete robot;
  robot = (RobotBase*)0;
  fdm   = (FDMBase*)0;
}


void ModRobotInterface::loadAirplane(const char* filename, 
                                     FDMEnviroment* myEnv,
                                     SimpleXMLTransfer* cfg)
{
  std::string notloadstring = "";
  Clean();
  
  // Try all available FDMs until one of them doesn't throw an exception:

  if (fdm == 0)
  {
    try
    {
      robot = new CRRC_AirplaneSim_Playback(filename);
      std::cout << "Using CRRC_AirplaneSim_Playback: " << filename << "\n";
    }
    catch (XMLException e)
    {
      notloadstring += "  Not CRRC_AirplaneSim_Playback: " + std::string(e.what()) + std::string("\n");
      robot = 0;
    }
  }  
  
  if (robot == 0)
  {
    std::cerr << "The file could not be loaded by any FDM.\n"
      << "They said:\n"
      << notloadstring;
  }
  fdm = robot;
}

RobotBase::RobotBase() : FDMBase("", (FDMEnviroment*)0)
{
}
