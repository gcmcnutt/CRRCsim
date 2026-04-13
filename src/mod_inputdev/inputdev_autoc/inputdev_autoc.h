/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2006, 2008 - Todd Templeton (original author)
 *   Copyright (C) 2008 - Jens Wilhelm Wulf
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
// Created 11/09/06 Todd R. Templeton <ttemplet@eecs.berkeley.edu>
// Based on tx_serial2.h

#ifndef TX_INTERFACE_AUTOC_H
#define TX_INTERFACE_AUTOC_H

#include "../inputdev.h"
#include "../../mod_misc/SimpleXMLTransfer.h"

#include "autoc/rpc/protocol.h"
#include "autoc/autoc.h"
#include "autoc/nn/evaluator.h"
#include "autoc/nn/serialization.h"
#include "autoc/eval/sensor_math.h"
#include "autoc/nn/nn_input_computation.h"
#include "autoc/eval/variation_generator.h"
#include "autoc/util/socket_wrapper.h"

#include <stdio.h>
#include <iostream>
#include <cmath>
#include <array>
#include <unistd.h>

using namespace std;

// #define DETAILED_LOGGING 1

#define FEET_TO_METERS 0.3048
#define EVAL_UPDATE_INTERVAL_MSEC_DEFAULT 100   // Sensor+NN cadence (~10Hz)
#define COMPUTE_LATENCY_MSEC_DEFAULT 30         // Bench-measured: consolidated MSP fetch(12)+eval(5)+send(12)=29ms
#define ENGAGE_DELAY_MSEC_DEFAULT 750           // Measured INAV MANUAL→autoc handoff delay (2026-04-07 flight)
#define SIM_FPS 25.0                            // ~40 Hz physics tick assumption for overflow calc

// ACRO mode rate PID — converts NN rate commands to surface deflections
// NN output [-1,1] → desired rate [-max_rate, +max_rate] rad/s
// PID: error = desired_rate - actual_rate (rad/s) → surface deflection [-1,1]
// All rate math in rad/s to match FDM and AircraftState convention.
//
// Max angular rates (deg/s config, converted to rad/s in PID code)
// Set to match actual FDM rate capability at full surface deflection.
// Real aircraft (Mar 27 flight): roll ~430°/s, pitch ~300°/s.
// These are NOT the INAV rate_param config values (560/400/240).
#define ACRO_MAX_RATE_ROLL  430.0               // flight measured max roll ≈ 7.50 rad/s
#define ACRO_MAX_RATE_PITCH 300.0               // flight measured max pitch ≈ 5.24 rad/s
#define ACRO_MAX_RATE_YAW   180.0               // estimated (no rudder, yaw from coupling)

// PID gains — empirical for CRRCSim FDM (NOT direct copies of INAV gains)
// INAV's gains are tuned for its own internal units and servo response.
// These must be strong enough that at ZERO command, P+I can counter
// aerodynamic coupling (sideslip→roll, etc.) to hold attitude.
// Rule of thumb: P * max_disturbance_rate / SCALE should produce
// enough surface deflection to counter the disturbance.
// FF: feedforward (proportional to desired rate)
// P:  proportional to rate error — must be large enough to hold attitude
// I:  integral of rate error — drives steady-state error to zero
#define ACRO_FF_ROLL   50.0
#define ACRO_P_ROLL    40.0
#define ACRO_I_ROLL    15.0
#define ACRO_FF_PITCH  50.0
#define ACRO_P_PITCH   40.0
#define ACRO_I_PITCH   15.0
#define ACRO_FF_YAW    60.0
#define ACRO_P_YAW     40.0
#define ACRO_I_YAW     15.0

// PID output scaling — divides PID sum to produce [-1,1] surface deflection
// At full roll (7.50 rad/s): FF*7.50/SCALE = 50*7.50/350 = 1.07 (clamped, full authority)
// At full pitch (5.24 rad/s): FF*5.24/SCALE = 50*5.24/350 = 0.75 (good authority + P/I headroom)
// Tune this to match FDM rate response. Too low = overshoot, too high = sluggish.
#define ACRO_PID_SCALE 350.0

// Settable at runtime (see inputdev_autoc.cpp).
extern unsigned long gEvalUpdateIntervalMsec;
extern unsigned long gComputeLatencyMsec;
unsigned long getCycleCounterOverflow();

class T_TX_InterfaceAUTOC : public T_TX_Interface
{
public:
  T_TX_InterfaceAUTOC();
  virtual ~T_TX_InterfaceAUTOC();

  /**
   * Get input method
   */
  virtual int inputMethod() { return (T_TX_Interface::eIM_autoc); };

  /**
   * Initialize interface. Read further config data from a file, if necessary.
   */
  int init(SimpleXMLTransfer *config);

  /**
   * Write configuration back
   */
  virtual void putBackIntoCfg(SimpleXMLTransfer *config);

  /**
   * Set current input data. If some value is not available, the value
   * is not overwritten.
   */
  void getInputData(TSimInputs *inputs);

private:
  unsigned short int callbackPort;
  TcpSocket *socket_ = nullptr;

  unsigned long lastUpdateTimeMsec = 0;
  unsigned long cycleCounter = 0;
  bool simCrashed = false;

  gp_scalar pitchCommand = 0;
  gp_scalar rollCommand = 0;
  gp_scalar throttleCommand = 0;

  // diagnostics ring buffer
  static constexpr int DIAG_BUFFER_SIZE = 15;
  std::array<unsigned long, DIAG_BUFFER_SIZE> diagBuffer{};
  int diagIndex = 0;
  int diagCount = 0;

  // here's the work to do and results
  bool evalDataEmpty = true;
  int priorPathSelector = -1;
  int pathSelector = 0;
  int pathIndex = 0;
  gp_scalar rabbitOdometer = 0.0f;
  gp_scalar crrcsimRabbitSpeed = 0.0f;   // Set from speed profile at path start — 0 = uninitialized
  std::vector<RabbitSpeedPoint> rabbitSpeedProfile;  // Time-varying profile (generated per path)
  RabbitSpeedConfig rabbitSpeedConfig = RabbitSpeedConfig::defaultConfig();
  EvalData evalData;
  EvalResults evalResults;
  std::vector<AircraftState> aircraftStates;
  std::vector<DebugSample> debugSamplesCurrentPath;
  double quatDotPast[4] = {0,0,0,0};
  int workerIndex = -1;
  int workerPid = 0;

  int evalCounter = 0;  // Total evaluations this worker has processed

  // Engage delay window — simulates INAV handoff delay (NN runs but stick
  // is held at {0,0,cruise_throttle} for the first N ticks of each scenario).
  int engageDelayTicksRemaining = 0;
  unsigned long engageDelayMsec = ENGAGE_DELAY_MSEC_DEFAULT;
  gp_scalar engageCoastThrottle = 0.0f;  // set per-scenario from entrySpeedFactor

  // NN controller
  NNGenome nnGenome;
  std::vector<Path> path;
};

#endif
