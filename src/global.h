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

    // VARIATIONS1: Entry and wind direction offsets from ScenarioMetadata
    // Set by inputdev_autoc when scenario changes, applied by init code
    static double entryHeadingOffset;    ///< radians, offset from config heading
    static double entryRollOffset;       ///< radians, initial roll attitude
    static double entryPitchOffset;      ///< radians, offset from config pitch
    static double entrySpeedFactor;      ///< multiplier on config velocity
    static double windDirectionOffset;   ///< radians, offset from config wind dir

    // Entry position offsets (see specs/005-entry-fitness-ramp)
    static double entryNorthOffset;      ///< meters, NED North position offset
    static double entryEastOffset;       ///< meters, NED East position offset
    static double entryAltOffset;        ///< meters, NED Down altitude offset

    // 034 US4 — craft variation parameters from ScenarioMetadata.craft*
    // (AFTER worker-side applyVariationScale ramping). Set by inputdev_autoc
    // at the per-scenario reset hook, consumed in fdm_larcsim init/engine.
    // CG/trim/drag/pitch-eff/roll-eff are additive deltas (0.0 = nominal);
    // craftThrustScale is a multiplier (1.0 = nominal).
    static double craftCGDelta;          ///< dimensionless CG arm offset (CRRCSim XML units, e.g. hb1_streamer nominal ≈ 0.28)
    static double craftDragDelta;        ///< fraction, CD_prof multiplier delta
    static double craftTrimDelta;        ///< radians, Cm_0 trim offset
    static double craftThrustScale;      ///< multiplier on engine maxThrust
    static double craftPitchEffDelta;    ///< fraction, pitch-authority multiplier delta
    static double craftRollEffDelta;     ///< fraction, roll-authority multiplier delta

    // 037 actuator dynamics (operator decision: in-FDM, substep dt).
    // Absolute physical parameters for the in-FDM actuator filter (servo
    // lag+slew on aileron/elevator, first-order thrust lag). Set per-scenario
    // by inputdev_autoc AFTER worker-side applyVariationScale ramping;
    // consumed each FDM substep in fdm_larcsim. Defaults = nominal centers so
    // a no-craft run still runs the nominal lag model.
    static double servoTau;              ///< s, servo first-order lag time constant — v2: UNUSED by the FDM (kept for wire/draw stability)
    static double servoSlew;             ///< /s, servo slew-rate limit, full-throw/s (v2 center ≈12.1 from the 0.07s/60° DSM-44 transit)
    static double thrustTau;             ///< s, thrust first-order lag time constant (center 0.150)

    // 037 servo v2 (operator 2026-06-11, post-t6/t7 A/B): datasheet-shaped
    // servo = 50 Hz PWM command LATCH (per-scenario phase = the 0-20 ms
    // dead-time) + pure slew toward the latched target. Gated by
    // servoModelEnabled (WorkerInit.servoModelEnabled <- ServoModelEnabled
    // ini knob) so A/B run pairs differ by ini only.
    static bool servoModelEnabled;       ///< master switch for the in-FDM servo block
    static double servoPwmPhase;         ///< s, per-scenario PWM latch phase, [0, 0.020)
};


/** This macro logs a line of text to the console */
#define LOG(_x)     do{                                               \
                        LogMessageEvent msg(_x);                      \
                        EventDispatcher::getInstance()->raise(&msg);  \
                      }while(0)

#endif //GLOBAL_H
