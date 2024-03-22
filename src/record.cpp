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

#include "record.h"

#include "mod_misc/filesystools.h"
#include "mod_misc/lib_conversions.h"
#include "mod_robots/robotfile.h"
#include <crrc_config.h>

#include <iostream>
#include <cerrno>
#include <cstring>

FlightRecorder::FlightRecorder(std::string output_directory)
{
  outdir = output_directory;
  state  = eNoFile;
  if (outdir.length())
    outdir += "/";
}

void FlightRecorder::Start(SimpleXMLTransfer* data)
{
  // insert mandatory data
  data->setAttribute("CRRCSim", PACKAGE_VERSION);  
  data->setName("CRRCSim_record");
  
  if (state == eRecording)
    Stop();
      
  // only use/keep four files for automatic storage:
  num = (num+1) & 0x03;
  //
  filename = "";
  out.open((outdir + "record" + itoStr(num, '0', 3) + ".crrclog_").c_str(), std::ios::binary);
  if (!out)
    std::cerr << "error opening logfile: " << strerror(errno) << "\n";
  
  data->print(out, 0);
  state = eRecording;
  descr = "";
}

void FlightRecorder::Stop()
{
  if (state == eRecording)
  {
    // Insert description
    SimpleXMLTransfer* data = new SimpleXMLTransfer();
    data->setName("descr");
    data->setContent(descr);
    InsertXML(data);
    delete data;
    
    // 
    out.close();
    
    // rename?
    if (filename.length() > 0)
    {
      std::string fn = "record" + itoStr(num, '0', 3) + ".crrclog_";
      FileSysTools::move(outdir+filename+".crrclog", outdir+fn);
    }
  }
  state = eNoFile;
}

void FlightRecorder::SetFilename(std::string newname)
{
  newname = trim(newname);
  if (newname.rfind(".crrclog") == newname.length()-8)
    filename = newname.substr(0, newname.length()-8);
  else
    filename = newname;
}
  
void FlightRecorder::InsertMarker(int data)
{
  if (state == eRecording)
  {
    const char rt = 0x02;
    out.write((char*)&rt, 1);
    RobotFile::WriteInt32(out, data);
  }
}

void FlightRecorder::InsertXML(SimpleXMLTransfer* data)
{
  if (state == eRecording)
  {
    const char rt = 0x03;
    out.write((char*)&rt, 1);
    data->print(out, 0);
  }
}

void FlightRecorder::AirplanePosition(double dt, int multiloop, FDMBase* fdm)
{
  if (state == eRecording)
  {
    const char rt = 0x00;
    out.write((char*)&rt, 1);

    RobotFile::WriteDouble(out, dt*multiloop);
    CRRCMath::Vector3 pos = fdm->getPos();
    RobotFile::WriteFloat(out, pos.r[0]);
    RobotFile::WriteFloat(out, pos.r[1]);
    RobotFile::WriteFloat(out, pos.r[2]);
    RobotFile::WriteInt16(out, fdm->getPhi()*ROBOT_EULER_TO_INT16);
    RobotFile::WriteInt16(out, fdm->getTheta()*ROBOT_EULER_TO_INT16);
    RobotFile::WriteInt16(out, fdm->getPsi()*ROBOT_EULER_TO_INT16);
  }
}

std::string FlightRecorder::GetFilename() 
{
  return(filename); 
}

