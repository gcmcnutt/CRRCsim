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

#include "robots.h"
#include "mod_misc/filesystools.h"
#include "mod_misc/ls_constants.h"
#include "mod_fdm/xmlmodelfile.h"
#include "mod_robots/robot.h"


#include <iostream>

Robots::Robots()
{
}

void Robots::AddRobot(std::string robotfilename)
{
  Robot* robot = new Robot();

  // 
  robot->fi = new ModRobotInterface();
  robot->fi->loadAirplane(robotfilename.c_str(), (FDMEnviroment*)0, (SimpleXMLTransfer*)0);  
  if (robot->fi->robot)
  {    
    SimpleXMLTransfer* header = robot->fi->robot->GetHeader();
    
    std::string filename = FileSysTools::getDataPath(header->getString("airplane.file"));
    
    SimpleXMLTransfer* xml = new SimpleXMLTransfer(filename);
    XMLModelFile::SetGraphics(xml, header->getInt("airplane.graphics"));
    
    // initialize the visual representation
    // first collect all relevant information from the model file
    std::string s;      
    s = XMLModelFile::getGraphics(xml)->getString("model");
        
    // Offset of center of gravity
    CRRCMath::Vector3  pCG;         
    pCG = CRRCMath::Vector3(0, 0, 0);
    if (xml->indexOfChild("CG") >= 0)
    {
      SimpleXMLTransfer* i;
      i = xml->getChild("CG");
      pCG.r[0] = i->attributeAsDouble("x", 0);
      pCG.r[1] = i->attributeAsDouble("y", 0);
      pCG.r[2] = i->attributeAsDouble("z", 0);
      
      if (i->attributeAsInt("units") == 1)
        pCG *= M_TO_FT;
    }
     
    // plib automatically loads the texture file, but it does not know which directory to use.
    // where is the object file?
    std::string of = FileSysTools::getDataPath("objects/" + s);
    // compile and set relative texture path
    std::string tp = of.substr(0, of.length()-s.length()-1-7) + "textures";
    
    robot->vis_id = Video::new_visualization(of, tp, pCG, xml);
  
    list.push_back(robot);
  }
}

void Robots::Update(double dt, int multiloop)
{
  TSimInputs dummy;
  for (unsigned int n=0; n<list.size(); n++)
  {
    list[n]->fi->update(&dummy, dt, multiloop);
    
    Video::set_position(list[n]->vis_id,
                        list[n]->fi->fdm->getPos(),
                        list[n]->fi->fdm->getPhi(),
                        list[n]->fi->fdm->getTheta(),
                        list[n]->fi->fdm->getPsi());
  }
}

void Robots::Reset()
{
  for (unsigned int n=0; n<list.size(); n++)
  {
    Video::reset_shadow(list[n]->vis_id);
    list[n]->fi->initAirplaneState(0, 0, 0, 0, 0, 0, 0);    
  }
}
  
void Robots::RemoveAll()
{
  for (unsigned int n=0; n<list.size(); n++)
  {
    Video::delete_visualization(list[n]->vis_id);
    delete list[n]->fi;
  }
  list.clear();
}

void Robots::AnnounceMarker(int id)
{
  for (unsigned int n=0; n<list.size(); n++)
  {
    list[n]->fi->robot->ReceiveMarker(id);
  }
}

