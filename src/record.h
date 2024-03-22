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

#ifndef RECORD_H
# define RECORD_H

#include <fstream>
#include <string>
#include "mod_misc/SimpleXMLTransfer.h"
#include "mod_fdm/fdm.h"

/**
 * Record airplane position, attitude, control inputs, settings, results, 
 * and whatever to a file for later playback.
 * 
 * Files are read by CRRC_AirplaneSim_Playback and RobotFile.
 * 
 * See documentation/record_playback/
 *
 * todo: all binary storage code needs to be reviewed regarding endianess
 * and other portability issues which I do not know about. Recorded files
 * should work on any platform!
 * 
 * @author Jens W. Wulf
 */
class FlightRecorder
{
public:
  
  /**
   * All files are stored to output_directory.
   */
  FlightRecorder(std::string output_directory);
  
  /**
   * Start a new log; stop a previous one if necessary.
   */
  void Start(SimpleXMLTransfer* data);
  
  /**
   * Stop and end a running log. Renames the file if
   * necessary.
   */
  void Stop();
  
  /**
   * Write timestep, position and attitude to file.
   */
  void AirplanePosition(double dt, int multiloop, FDMBase* fdm);
  
  /**
   * Insert some marker.
   */
  void InsertMarker(int data);
  
  /**
   * Insert arbitrary data into the file.
   */
  void InsertXML(SimpleXMLTransfer* data);
  
  /**
   * Set the filename to which the current file is
   * renamed to on Stop()
   */
  void SetFilename(std::string newname);
  
  /**
   * returns the current filename
   */
  std::string GetFilename();
  
  /**
   * Description: will be saved when the file is closed
   */
  std::string descr;
  
    
private:    
  std::string filename;
  
  /**
   * ofstream in use
   */
  std::ofstream out;
  
  /**
   * All files are stored to this output directory
   */
  std::string outdir;
  
  enum eState { eNoFile, eRecording };
  eState state;
  int num;
};

#endif
