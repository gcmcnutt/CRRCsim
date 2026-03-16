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
#define COMPUTE_LATENCY_MSEC_DEFAULT 40         // NN compute + scheduling latency
#define SIM_FPS 25.0                            // ~40 Hz physics tick assumption for overflow calc

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
  EvalData evalData;
  EvalResults evalResults;
  std::vector<AircraftState> aircraftStates;
  std::vector<DebugSample> debugSamplesCurrentPath;
  double quatDotPast[4] = {0,0,0,0};
  int workerIndex = -1;
  int workerPid = 0;

  int evalCounter = 0;  // Total evaluations this worker has processed

  // NN controller
  NNGenome nnGenome;
  std::vector<Path> path;
};

#endif
