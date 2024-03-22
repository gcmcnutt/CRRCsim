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

#include "fdm_playback.h"
#include "marker.h"
#include "robotfile.h"

#include <math.h>
#include <iostream>


void CRRC_AirplaneSim_Playback::update(TSimInputs* inputs,
                                       double      dt,
                                       int         multiloop)
{
  if (eF3FState != eF3F_WaitForUser)
  {
    CRRCMath::Vector3 v3PosNew;
    
    dDeltaT -= dt * multiloop;
    
    while ((dDeltaT < 0 || eF3FState == eF3F_Jump) && !infile.eof() && eF3FState != eF3F_WaitForUser)
    {
      int marker;
      do 
      {
        infile.read(buf, 1);
        switch (buf[0])
        {
          case 0x02: // marker            
            marker = RobotFile::ReadInt32(infile);
            
            switch (marker)
            {
              case RECMARK_F3F_START:
                switch (eF3FState)
                {
                  case eF3F_Prep:
                    eF3FState = eF3F_WaitForUser;
                    break;
                    
                  default:
                    eF3FState = eF3F_Done;
                    break;
                }
                break;
                
              default: // ignore
                break;
            }
            break;
            
          case 0x03: // xml
            {
              SimpleXMLTransfer* data = 0;
              try
              {
                data = new SimpleXMLTransfer(infile);
              }
              catch (XMLException e)
              {
                data = 0;
              }
              // todo: tell someone about this data...
              if (data)
                delete data;
              // skip trailing '\n'
              infile.read(&(buf[1]), 1);
            }
            break;
            
          case 0x00:
            break;
            
          default:
            std::cerr << "unknown record type: " << (int)(buf[0]) << "\n";
            break;
        }  
      }
      while (buf[0] != 0 && !infile.eof());
      
      if (!infile.eof())
      {
        double timestep = RobotFile::ReadDouble(infile);
        if (eF3FState != eF3F_Jump)
          dDeltaT += timestep;
        v3PosNew.r[0] = RobotFile::ReadFloat(infile);
        v3PosNew.r[1] = RobotFile::ReadFloat(infile);
        v3PosNew.r[2] = RobotFile::ReadFloat(infile);
        v3Euler.r[0] = RobotFile::ReadInt16(infile) / ROBOT_EULER_TO_INT16;
        v3Euler.r[1] = RobotFile::ReadInt16(infile) / ROBOT_EULER_TO_INT16;
        v3Euler.r[2] = RobotFile::ReadInt16(infile) / ROBOT_EULER_TO_INT16;
        
        if (fFirstPos)
        {
          v3PosOld = v3PosNew;
          fFirstPos = false;
        }
        
        // Interpolate position. v3Euler can't be interpolated; this would 
        // be possible if a quaternion had been used, but is far too much 
        // trouble.
        // v3PosNew is the current position, current time is dDeltaT.
        // v3PosOld is the position of timestep ago. We need the position
        // at time=zero.
        v3Pos = v3PosOld + (v3PosNew - v3PosOld) * ((timestep-dDeltaT)/timestep);
        v3PosOld = v3PosNew;
      }
    }
  }
}

CRRC_AirplaneSim_Playback::CRRC_AirplaneSim_Playback(const char* filename) : RobotBase()
{
  header = 0;
  infile.open(filename, std::ios::binary);
  if (!infile)
  {
    throw XMLException("error opening infile");
  }
  // Read mandatory XML header
  header = new SimpleXMLTransfer(infile);
  // skip trailing '\n'
  infile.read(buf, 1);
  
  if (header->getName().compare("CRRCSim_record") != 0)
  {
    throw XMLException("wrong file format");
  }
  this->filename = filename;
}


CRRC_AirplaneSim_Playback::~CRRC_AirplaneSim_Playback()
{
  if (header)
    delete header;
  infile.close();
}

void CRRC_AirplaneSim_Playback::initAirplaneState(double dRelVel,
                                                  double dPhi,
                                                  double dTheta,
                                                  double dPsi,
                                                  double X,
                                                  double Y,
                                                  double Z,
                                                  double R_X,
                                                  double R_Y,
                                                  double R_Z)
{
  dDeltaT = 0;
  if (infile.eof())
  {
    infile.close();
    infile.open(filename.c_str());
  }
  else
    infile.seekg(0);
  // read XML header
  SimpleXMLTransfer* dummy = new SimpleXMLTransfer(infile);
  delete dummy;
  // skip trailing '\n'
  infile.read(buf, 1);

  eF3FState = eF3F_Off;
  fFirstPos = true;
}

void CRRC_AirplaneSim_Playback::ReceiveMarker(int id)
{
  switch (id)
  {
    case RECMARK_F3F_RESET:
      eF3FState = eF3F_Prep;      
      break;
      
    case RECMARK_F3F_START:
      if (eF3FState == eF3F_Prep)
        // fast forward: read file until marker is found
        eF3FState = eF3F_Jump;
      else
        eF3FState = eF3F_Done;
      break;
  }
}
