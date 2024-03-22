/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2008 Olivier Bordes (original author)
 * Copyright (C) 2005, 2006 Jan Reucker
 * Copyright (C) 2010 Jens Wilhelm Wulf
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

/**
 * \file T_GameHandler.h
 *
 * General interface for Game mode
 */
#ifndef __TGAMEHANDLER_H
#define __TGAMEHANDLER_H

 
#include "../include_gl.h"
#include <iostream>
#include <string>

#include "../mod_misc/SimpleXMLTransfer.h"

class Recorder;
class Robots;

class T_GameHandler
{
  public:
    
    T_GameHandler() {};
    virtual ~T_GameHandler() {};
  
    /**
     * Disable this game mode (because users has enabled another one)
     */
    virtual void disable() { return; };

    /**
     *  Calculate game-specific stuff with respect to the
     *  current absolute time (SDL_GetTicks()) and the
     *  aircraft's position and orientation.
     *  It is possible to write additional data into the flight
     *  record file.
     */
    virtual void update(float a,float b, float c, FlightRecorder* recorder, Robots* robots) {};

    /**
     *  draw game-specific stuff, like F3F turn markers or
     *  a game-specific text overlay
     */
    virtual void draw() {};
    
    /**
     *  draw a game-specific 2D overlay (informal text etc.)
     */
    virtual void display_infos(GLfloat w, GLfloat h){};

    /* game reset */
    virtual void reset() {};

    virtual std::string gameType() const {return "default"; };

    /**
     * The game mode returns a header which is written at the beginning of
     * a recorded file.
     */
    virtual SimpleXMLTransfer* GetRecordHeader() { return(new SimpleXMLTransfer()); };

    /**
     * The game mode returns the center and the angular extent of the game scene.
     * Returns 0 if parameters have not been set
     */
    virtual int GetSmartCameraPar(float& phi_w0, float& phi_h0, float& range_w, float& range_h) { return 0; };
};

#endif
