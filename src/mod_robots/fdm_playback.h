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

#ifndef FDM_PLAYBACK_H
# define FDM_PLAYBACK_H

#include "robot.h"

/**
 * This is not really a FDM, but reads position, attitude and more from
 * a file which has previously been recorded by FlightRecorder (see record.h).
 * It can be used for flight playback (demo mode) and shadow mode.
 * 
 * See documentation/record_playback/
 * 
 * It knows about F3F mode in order to sync playback to the user's
 * F3F run in shadow mode.
 * 
 * todo: all binary storage code needs to be reviewed regarding endianess
 * and other portability issues which I do not know about. Recorded files
 * should work on any platform!
 *
 * @author Jens Wilhelm Wulf
 */
class CRRC_AirplaneSim_Playback : public RobotBase
{
public:
  
  virtual void initAirplaneState(double dRelVel,
                                 double dPhi,
                                 double dTheta,
                                 double dPsi,
                                 double X,
                                 double Y,
                                 double Z,
                                 double R_X = 0.0,
                                 double R_Y = 0.0,
                                 double R_Z = 0.0);
  
  virtual void update(TSimInputs* inputs,
                      double      dt,
                      int         multiloop);
  
  /**
   * read from file
   */
  CRRC_AirplaneSim_Playback(const char* filename);
  
  
  virtual ~CRRC_AirplaneSim_Playback();

  /**
   *
   */
  virtual void ReceiveMarker(int id);
  
private:
  enum enum_F3FState { eF3F_Off, eF3F_Prep, eF3F_WaitForUser, eF3F_Jump, eF3F_Done};
  enum_F3FState eF3FState;
  
  std::string filename;
  std::ifstream infile;
  double dDeltaT;
  CRRCMath::Vector3 v3PosOld;
  bool fFirstPos;
  
  char buf[7*sizeof(double)+1];
};

#endif
