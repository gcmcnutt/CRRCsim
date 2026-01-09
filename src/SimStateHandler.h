/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2006 Jan Reucker (original author)
 * Copyright (C) 2006 Todd Templeton
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
  

/** \file SimStateHandler.cpp
 *
 *  This file includes the definition of the class
 *  SimStateHandler, which takes care of the current
 *  simulation state (running, paused, ...) and provides
 *  some handy time values.
 *
 *  \author J. Reucker
 */

#ifndef SIMSTATEHANDLER_H
#define SIMSTATEHANDLER_H

#include "mod_fdm/fdm_inputs.h"
#include "mod_main/EventDispatcher.h"
#include <SDL.h>

// Typedef section :

/// Function pointer to the "idle" function
typedef void (*TIdleFuncPtr)(TSimInputs*, int);

/// simulation state definitions
typedef enum
{
  STATE_RESUMING,     // the next iteration will be resuming from a pause
  STATE_PAUSED,       // the simulation is paused
  STATE_RUN,          // the simulation is running
  STATE_CRASHED,      // the plane has crashed
  STATE_EXIT          // exit after next iteration
} T_SimState;  


/*****************************************************************************/
// Classes section :

/** \brief The simulation state handler
 *
 *  This class represents the simulation's state machine.
 *  It also provides some handy time values (since startup,
 *  in-game-time, ...).
 *  
 */
class SimStateHandler : public EventListener
{
  private:
    T_SimState    nState;         ///< state of the simulation
    TIdleFuncPtr  IdleFunc;       ///< the currently used "idle function"
    TIdleFuncPtr  OldIdleFunc;    ///< stores idle pointer for temporary switching
    unsigned long int sim_steps;  ///< number of simulation steps performed
    unsigned long int pause_time; ///< time when pause mode was entered
    unsigned long int accum_pause_time; ///< pause time since last reset
    unsigned long int reset_time; ///< time of the last reset
  
    /// Handle a crash
    void crash();
  
  public:
    /// The constructor
    SimStateHandler();
    
    /// The destructor
    ~SimStateHandler();
  
    /// pause the simulation
    void pause();
  
    /// resume from pause mode
    void resume();

    /// reset the simulation
    void reset();

    /// reset the simulation with a specific wind seed (for autoc mode)
    void reset(unsigned int windSeed);
  
    /// quit the simulation
    void quit();
  
    /// query the current state
    T_SimState getState()  const {return nState;};
    
    /// set state machine to a new state
    void setState(T_SimState new_state) {nState = new_state;};
    
    /// run the current "idle" function
    void doIdle(TSimInputs* in, int deltaT);
    
    /// temporarily switch to a different idle function
    void setNewIdle(TIdleFuncPtr new_idle);
    
    /// fall back to previous idle function
    void resetIdle();

    /// increase number of simulation steps
    void incSimSteps(int multiloop) {sim_steps += multiloop;};
    
    /// get the total time since the sim was launched (in ms)
    unsigned long int getTotalTime() const {return SDL_GetTicks();};
    
    /// get the time since the last reset (including pause time, in ms)
    unsigned long int getTotalTimeSinceReset();
    
    /// get the time since the last reset (not counting pause time, in ms)
    unsigned long int getGameTimeSinceReset();

    /// get the simulation time since the last reset (number of sim steps * dt, in ms)
    unsigned long int getSimulationTimeSinceReset();
    
    /// interface to the EventDispatcher
    void operator()(const Event* ev);
};

#endif  // SIMSTATEHANDLER_H

