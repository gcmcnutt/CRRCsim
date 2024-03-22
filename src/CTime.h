/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005 Olivier Bordes (original author)
 * Copyright (C) 2005 Jan Reucker
 * Copyright (C) 2005, 2008 Jens Wilhelm Wulf
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
  

#ifndef C_TIME_H
#define C_TIME_H

#include <SDL.h>
#include "mod_misc/SimpleXMLTransfer.h"

#define DEFAULT_GAME_SPEED  (60)  ///< default frames-per-second

/**
 * This class calls SDL_Delay() in order to not consume too much 
 * CPU cycles as long as the frame rate is high enough.
 */
class CTime
{
  public:

    CTime(SimpleXMLTransfer *cfg);
    ~CTime();

    void setGameSpeed(Uint16 speed);
    int  update();
    void putBackIntoCfg(SimpleXMLTransfer *cfg);
    int  getGameFPS() {return (int)(fFPS);};

  private:
    Uint16 gameSpeed;         ///< the desired game speed in frames/s
    Uint16 cycleLength;
    Uint32 timer1;
    Uint32 lastFPSTime;
    int frames;
    float fFPS;               ///< actual game speed in frame/s
    float dtMillis;           ///< integration time (Global::dt) in milliseconds
};

#endif  // C_TIME_H
