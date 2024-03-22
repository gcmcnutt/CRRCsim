/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2008 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2005 - Jan Reucker
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
#include "inputdev_rctran2.h"

#include <stdio.h>
#include <stdlib.h>


inline float convVal(int value)
{
  if (value > 2000)
    return(0.5);
  else if (value < 1000)
    return(-0.5);
  else
    return((value-1500)/1000.0);
}

T_TX_Interface_RCTran2::T_TX_Interface_RCTran2()
  : cname("rctran2")
{
}

T_TX_Interface_RCTran2::~T_TX_Interface_RCTran2()
{
}

int T_TX_Interface_RCTran2::init(SimpleXMLTransfer* config)
{
  T_TX_InterfacePPM::init(config);

  filenameToReadFrom = config->getString("inputMethod.rctran2.file", "/proc/rctran");
  
  mixer = new T_TX_Mixer(this, config, "inputMethod.rctran2");
  
#ifdef linux
  rctran = fopen(filenameToReadFrom.c_str(), "r");
  if (rctran != NULL)
  {
    fclose(rctran);
    return(0);
  }
  else
  {
    errMsg  = "Error opening input file\n";
    errMsg += filenameToReadFrom;
    return(-1);
  }
#else
  rctran = NULL;
  errMsg = "This interface only works on linux.";
  return(-1);
#endif
}

void T_TX_Interface_RCTran2::getInputData(TSimInputs* inputs)
{
  // read values from file
  int nVal[8];
  
  rctran = fopen(filenameToReadFrom.c_str(), "r");  
  int cnt = fscanf(rctran, "%i,%i,%i,%i,%i,%i,%i,%i", &nVal[0], &nVal[1], &nVal[2],
                   &nVal[3], &nVal[4], &nVal[5], &nVal[6], &nVal[7]);  
  fclose(rctran);

  // write data to 'rc_channel_values'
  for (int n=0; n<cnt; n++)
  {
    rc_channel_values[n] = convVal(nVal[n]);
  }

  T_TX_InterfacePPM::getInputData(inputs);
}

void T_TX_Interface_RCTran2::putBackIntoCfg(SimpleXMLTransfer* config)
{
  T_TX_InterfacePPM::putBackIntoCfg(config);

  config->setAttributeOverwrite("inputMethod.rctran2.file", filenameToReadFrom);
  mixer->putBackIntoCfg(config);
}

