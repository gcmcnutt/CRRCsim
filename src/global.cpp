/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2006 - 2009 Jan Reucker (original author)
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
  

/** \file global.cpp
 *
 *  Initialization of the global variables.
 */
 
#include "global.h"
#include <stdlib.h>

int               Global::training_mode = 0;
int               Global::nVerbosity = 0;
int               Global::HUDCompass = 0;
int               Global::windVectors = 0;
int               Global::modelView = 0;
int               Global::testmode = 0;
int               Global::wind_mode = 2;
int               Global::slowMotion = 0;
float             Global::dt;
float             Global::slowTimeScale = 2.0;
std::string       Global::verboseString;
SimStateHandler*  Global::Simulation = NULL;
Scenery*          Global::scenery = NULL;
CGUIMain*         Global::gui = NULL;
CRRCAudioServer*  Global::soundserver = NULL;
T_GameHandler*    Global::gameHandler = NULL;
TSimInputs        Global::inputs;
T_TX_Interface*   Global::TXInterface; 
TInputDev*        Global::inputDev;
Aircraft*         Global::aircraft;
FlightRecorder*   Global::recorder;
Robots*           Global::robots;
