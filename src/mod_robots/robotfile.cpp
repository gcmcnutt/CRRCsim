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
#include "robotfile.h"

#include <iostream>

RobotFile::RobotFile(std::string filename)
{
  char buf[7*sizeof(double)+1];
  SimpleXMLTransfer* tmp;
  std::ifstream infile;
  
  infile.open(filename.c_str(), std::ios::binary);
  
  try
  {
    // Read mandatory XML header
    tmp = new SimpleXMLTransfer(infile);
    xmls.push_back(tmp);
    // skip trailing '\n'
    infile.read(buf, 1);
    
    do 
    {
      infile.read(buf, 1);
      switch (buf[0])
      {
        case 0x00:
          infile.read(&(buf[1]), 8+3*4+3*2);
          break;
          
        case 0x02: // marker
          RobotFile::ReadInt32(infile);
          break;
          
        case 0x03: // xml
          tmp = new SimpleXMLTransfer(infile);
          // skip trailing '\n'
          infile.read(&(buf[1]), 1);
          xmls.push_back(tmp);
          break;
          
        default:
          std::cerr << "unknown record type: " << (int)(buf[0]) << "\n";
          break;
      }  
    }
    while (!infile.eof());
  }
  catch (XMLException e)
  {
  }
  infile.close();
}

RobotFile::~RobotFile()
{
  for (unsigned int n=0; n<xmls.size(); n++)
    delete xmls[n];
}

std::string RobotFile::ReadDescription()
{
  for (unsigned int n=0; n<xmls.size(); n++)
    if (xmls[n]->getName().compare("descr") == 0)
      return(xmls[n]->getContentString());
  return("");
}

