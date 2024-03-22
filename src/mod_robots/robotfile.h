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
#ifndef ROBOTFILE_H
# define ROBOTFILE_H

# include <fstream>
# include <vector>
# include <string>
# include "../mod_misc/SimpleXMLTransfer.h"

#define ROBOT_EULER_TO_INT16 (32767.0/2/M_PI)

/**
 * This reflects some parts of the specification of a mixed xml/binary file describing a
 * fligth record or robot plane. There doesn't have to be a binary part!
 * 
 * todo: care about endianess!
 * 
 * @author Jens Wilhelm Wulf
 */
class RobotFile
{
public:
  
  RobotFile(std::string filename);
  
  ~RobotFile();
  
  std::string ReadDescription();
  
  /**
   * 
   */
  static inline int ReadInt32(std::ifstream& in)
  {
    int nVal;
    in.read((char*)&nVal, 4);
    return(nVal);
  }
  
  /**
   * 
   */
  static inline int ReadInt16(std::ifstream& in)
  {
    int nVal = 0;
    in.read((char*)&nVal, 2);
    if ((nVal & 0x8000) != 0)
      nVal |= 0xFFFF0000;
    return(nVal);
  }
  
  /**
   * returns double!
   */
  static inline double ReadFloat(std::ifstream& in)
  {
    float fVal;
    in.read((char*)&fVal, 4);
    return(fVal);
  }
  
  /**
   * 
   */
  static inline double ReadDouble(std::ifstream& in)
  {
    double dVal;
    in.read((char*)&dVal, 8);
    return(dVal);
  }
  
  /**
   * store double as double
   */
  static inline void WriteDouble(std::ofstream& out, double dVal)
  {
    // todo: endianess?
    out.write((char*)&dVal, 8);
  }
  
  /**
   * store double as float 
   */
  static inline void WriteFloat(std::ofstream& out, double dVal)
  {
    float fVal = dVal;
    // todo: endianess?
    out.write((char*)&fVal, 4);
  }
  
  /**
   * 
   */
  static inline void WriteInt32(std::ofstream& out, int nVal)
  {
    out.write((char*)&nVal, 4);
  }
  
  /**
   * 
   */
  static inline void WriteInt16(std::ofstream& out, double dVal)
  {
    int nVal = (int)(dVal+0.5);
    out.write((char*)&nVal, 2);
  }
  
private:
  std::vector<SimpleXMLTransfer*> xmls;
};
#endif
