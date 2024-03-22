/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2006, 2008, 2009 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2005, 2006, 2008 - Jan Reucker
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
#include "battery.h"

#include <iostream>
#include <fstream>
#include <cstdlib>
#include "../../mod_misc/filesystools.h"
#include "../../mod_misc/lib_conversions.h"

Power::Battery::Battery()
{
  shafts.push_back(new Shaft());
}

Power::Battery::Battery(SimpleXMLTransfer* xml)
{
  // create children
  {
    SimpleXMLTransfer* shaft;
    
    for (int n=0; n<xml->getChildCount(); n++)
    {
      shaft = xml->getChildAt(n);
      if (shaft->getName().compare("shaft") == 0)
        shafts.push_back(new Shaft(shaft));
    }
  }
}

void Power::Battery::ReloadParams(SimpleXMLTransfer* xml)
{
  throttle_min = xml->getDouble("throttle_min");
  
  SimpleXMLTransfer* bat;
  bool               fExtern = true;
  if (xml->indexOfAttribute("filename") >= 0)
    bat = new SimpleXMLTransfer(FileSysTools::getDataPath("models/battery/" + xml->getString("filename") + ".xml", true));
  else
  {
    bat     = xml;
    fExtern = false;
  }
  C_0   = bat->getDouble("C")*60*60;
  R_I   = bat->getDouble("R_I");
  U_off = bat->getDouble("U_off");
  
  std::cout << "Battery    : C=" << C_0/(60*60) << " Ah, R_I=" << R_I << " Ohm, U_off=" << U_off << "V\n";
  
  // voltage over capacity
  {
    double                 U_0 = bat->getDouble("U_0");
    std::string            tab = bat->getChild("U_0rel")->getContentString();
    std::string::size_type pos;
    std::string::size_type start = 0;
    double                 U;
    
    voltage.clear();
    
    while ( (pos = tab.find(';', start)) != std::string::npos && start < tab.length())
    {
      U = U_0*atof(tab.substr(start, pos-start).c_str());
      std::cout << "         " << U << " V\n";
      voltage.push_back(U);
      start = pos+1;
    }
    
    // Calculate factor for easy interpolation:
    // There are size values:
    int size = voltage.size();
    // So I want to have an index in the range 0..(size-2). 
    // C_0 has to be converted to (size-1)*2^10.
    dInterpFact = ((size-1)<<10)/C_0;
  }
  
  if (fExtern)
    delete bat;
  
  // children
  {
    SimpleXMLTransfer* shaft;
    int nShaftCnt = 0;
    
    for (int n=0; n<xml->getChildCount(); n++)
    {
      shaft = xml->getChildAt(n);
      if (shaft->getName().compare("shaft") == 0)
        shafts[nShaftCnt++]->ReloadParams(shaft);
    }
  }
}

void Power::Battery::ReloadParams_automagic(SimpleXMLTransfer* xml)
{
  SimpleXMLTransfer* mydescr = xml->getChild("battery");
  
  // automagic configuration works bottom up:
  shafts[0]->ReloadParams_automagic(xml);
  
  R_I   = 0; 
  U_off = 1;
  U     = mydescr->getDouble("shaft.engine.automagic.U");
  C_0   = mydescr->getDouble("shaft.engine.automagic.I")*mydescr->getDouble("automagic.T");
  mydescr->setAttribute("C",     doubleToString(C_0/60/60));
  mydescr->setAttribute("U_0",   doubleToString(U));
  mydescr->setAttribute("U_off", doubleToString(U_off));
  mydescr->setAttribute("R_I",   doubleToString(R_I));

  throttle_min = mydescr->getDouble("throttle_min");
  {
    SimpleXMLTransfer* g = new SimpleXMLTransfer();
    g->setName("U_0rel");
    g->setContent("\n   1.00;\n   1.00;\n");
    mydescr->addChild(g);
  }
  voltage.clear();
  voltage.push_back(U);
  voltage.push_back(U);
  
  // Calculate factor for easy interpolation:
  // There are size values:
  int size = voltage.size();
  // So I want to have an index in the range 0..(size-2). 
  // C_0 has to be converted to (size-1)*2^10.
  dInterpFact = ((size-1)<<10)/C_0;  
}

Power::Battery::~Battery()
{
  // deallocate shafts
  for (unsigned int i = 0; i < shafts.size(); i++)
  {
    delete shafts[i];
    shafts[i] = NULL;
  }
}

void Power::Battery::step(PowerValuesStep* values)
{
  int          size = shafts.size();
  int          idx;
  double       thr;

  thr = values->inputs->throttle;
  
  if (thr < throttle_min && throttle_old > 0)
  {
    thr                      = throttle_min;
    values->inputs->throttle = throttle_min;
  }
  throttle_old = thr;
  
  if (U < U_off || C.val <= 0 || nUOffStatus == 1)
  {
    if (nUOffStatus == 1 && thr < 0.05)
      nUOffStatus = 0;
    else
      nUOffStatus = 1;
    values->U = 0;
  }
  else  
    values->U = U;
  
  values->I = 0;  
  
  for (int n=0; n<size; n++)
  {
    shafts[n]->step(values);
  }
  
  C.step(values->dt, -1*values->I);
  if (C.val < 0)
    C.init(0, 0);  
  
  {
    double dCapLeftRel = C.val/C_0;
    
    if (values->dBatCapLeftMin > dCapLeftRel)
      values->dBatCapLeftMin = dCapLeftRel;
  }
  
  // size of table:
  size = voltage.size();
  // max. index for interpolation:
  int idxmax = ((size-1)<<10)-1;
  // calc index
  idx  = (int)(dInterpFact * (C_0-C.val));
  if (idx > idxmax)
    idx = idxmax;
  // interpolation
  int idxh = idx >> 10;
  int idxl = idx & ((1<<10)-1);
  
  U = voltage[idxh] + (idxl * (voltage[idxh+1]-voltage[idxh]))*(1.0/1024);
  
//  std::cout << idxh << " " << U << " V, " << voltage[idxh] << " V, " << values->I  << " A\n";
  
  // voltage drop because of internal resistance
  U -= R_I * values->I;
}

void Power::Battery::showCapacity()
{
  std::cout << C.val/(60*60) << " Ah\n";
}

void Power::Battery::InitStates(CRRCMath::Vector3 vInitialVelocity)
{
  // set initial state
  U = voltage[0];
  throttle_old = 0;  
  nUOffStatus = 0;
  C.init(C_0, 0);
  
  for (unsigned int n=0; n<shafts.size(); n++)
    shafts[n]->InitStates(vInitialVelocity);
}
