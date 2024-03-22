/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2006-2009 Jan Reucker (original author)
 * Copyright (C) 2006 Todd Templeton
 * Copyright (C) 2007, 2008, 2010 Jens Wilhelm Wulf
 * Copyright (C) 2008 Olivier Bordes
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
  

/** \file global.h
 *
 *  Global variables used in CRRCsim.
 */

#ifndef GLOBAL_H
#define GLOBAL_H

#include <string>
#include "mod_fdm/fdm_inputs.h"
#include "mod_inputdev/inputdev.h"
#include "mouse_kbd.h"

// needed to make LOG() work without add. headers
#include "mod_main/EventDispatcher.h"

// There's no need to pull in the full headers here.
// Just declare the classes and leave the responsibility
// to the files that really need the single variables.
class SimStateHandler;
class CGUIMain;
class CRRCAirplane;
class CRRCAudioServer;
class ModFDMInterface;
class Scenery;
class T_GameHandler;
class T_TX_Interface;
class TInputDev;
class Aircraft;
class FlightRecorder;
class Robots;

class Global
{
  public:
    static int              training_mode;  ///< Draw thermals in the sky?
    static int              nVerbosity;     ///< How much info in the HUD?
    static int              HUDCompass;     ///< Draw azimuth/elevation in the HUD?
    static int              windVectors;    ///< Draw wind vectors in the sky?
    static int              modelView;      ///< Draw model-view window?
    static int              testmode;       ///< Test mode?
    static int              wind_mode;      ///< Wind estimation mode
    static int              slowMotion;     ///< Active slow-motion mode?
    static float            dt;             ///< time interval of integration of EOMs
    static float            slowTimeScale;  ///< Slow-motion time scale
    static std::string      verboseString;  ///< Informational line of text
    static SimStateHandler* Simulation;     ///< The simulation's main state machine.
    static Scenery*         scenery;        ///< The scenery.
    static CGUIMain*        gui;            ///< The GUI.
    static CRRCAudioServer* soundserver;    ///< The sound server.
    static T_GameHandler*   gameHandler;    ///< The active game mode.
    static TSimInputs       inputs;         ///< Control input values.
    static T_TX_Interface*  TXInterface; 
    static TInputDev*       inputDev;
    static Aircraft*        aircraft;       ///< A complete Aircraft (model & FDM).
    static FlightRecorder*  recorder;
    static Robots*          robots;
};


/** This macro logs a line of text to the console */
#define LOG(_x)     do{                                               \
                        LogMessageEvent msg(_x);                      \
                        EventDispatcher::getInstance()->raise(&msg);  \
                      }while(0)

#endif //GLOBAL_H
