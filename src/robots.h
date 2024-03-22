/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2010 - Jens Wilhelm Wulf (original author)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty off
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#ifndef ROBOTS_H
# define ROBOTS_H

#include <string>
#include "global_video.h"

class ModRobotInterface;

/**
 * data for one robot
 * 
 * @author Jens W. Wulf
 */
class Robot
{
public:
  ModRobotInterface* fi;
  long vis_id;
};

/**
 * Handles the list of robots (adding, removing, calling their
 * state update, updating their display, ..)
 * 
 * See documentation/record_playback/
 *
 * @author Jens W. Wulf
 */
class Robots
{
public:
  
  /**
   * 
   */
  Robots();
  
  /**
   * call this to load and add a robot
   */
  void AddRobot(std::string robotfilename);
  
  void RemoveAll();
  
  void Update(double dt, int steps);
  
  void Reset();
  
  /**
   * Call this to send a marker info to all robots
   */
  void AnnounceMarker(int id);
  
private:
  std::vector<Robot*> list;
};

#endif
