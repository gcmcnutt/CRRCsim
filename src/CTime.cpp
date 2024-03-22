/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005 Olivier Bordes (original author)
 * Copyright (C) 2005 Jan Reucker
 * Copyright (C) 2008 Jens Wilhelm Wulf
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
  

// handle the game timing to use CPU as appropriate
//

#define FPS_AVERAGING_TIME  1000  // average fps over this time (in milliseconds)

#include "CTime.h"
#include "global.h"
#include "SimStateHandler.h"
#include <iostream>

CTime::CTime(SimpleXMLTransfer *cfg)
{
  try
  {
    int speed;
    SimpleXMLTransfer *video = cfg->getChild("video", true);
    speed = video->attributeAsInt("fps", DEFAULT_GAME_SPEED);
    setGameSpeed(speed);
  }
  catch (XMLException e)
  {
    std::cerr << "CTime::CTime(SimpleXMLTransfer *cfg)" << std::endl;
    std::cerr << "XMLException: " << e.what() << std::endl;
    std::cerr << "Falling back to desired framerate of ";
    std::cerr << DEFAULT_GAME_SPEED << " FPS" << std::endl;
    setGameSpeed(DEFAULT_GAME_SPEED);
  }
}

CTime::~CTime()
{
}

void CTime::setGameSpeed(Uint16 speed)
{
  if (speed <= 0)
  {
    speed = DEFAULT_GAME_SPEED;
  }
  gameSpeed = speed;
  // to achieve maximum display smoothness try to set cycle length
  // as an integer multiple of Global::dt, to ensure that actual 
  // time step between frames is constant
  dtMillis = 1000.*Global::dt;
  cycleLength = (int)(1000/gameSpeed/dtMillis)*dtMillis + 0.5;
  timer1 = Global::Simulation->getTotalTime();
  
  // init FPS evaluation
  frames = 0;
  lastFPSTime = timer1;
  fFPS = 0.0;
}

int CTime::update()
{
  Uint32 time;
  int nDeltaTicks;
  
  // ensure we are not going too fast
  while (1)
  {
    SDL_Delay(1);   // delay 1 milliseconds
    time = Global::Simulation->getTotalTime();
    nDeltaTicks = time - timer1;
    if (nDeltaTicks >= cycleLength)
      break;
  }
  
  // update timing
  timer1 += nDeltaTicks;

  // keep track of frame count and wall clock time and update fps each second
  frames++;
  if (time - lastFPSTime > FPS_AVERAGING_TIME)
  {
    fFPS = frames*1000./(time - lastFPSTime);
    lastFPSTime = time;
    frames = 0;
  }

  // return the number of ticks (millisecond) to advance the game simulation
  return nDeltaTicks;
}

void CTime::putBackIntoCfg(SimpleXMLTransfer *cfg)
{
    SimpleXMLTransfer *video = cfg->getChild("video");
    video->setAttributeOverwrite("fps", gameSpeed);
}
