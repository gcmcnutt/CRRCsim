/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2006, 2008 - Todd Templeton (original author)
 *   Copyright (C) 2008 - Jan Reucker
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

#include "../../crrc_main.h"
#include "../../global.h"
#include "../../aircraft.h"
#include "../../SimStateHandler.h"
#include "../../mod_fdm/fdm.h"
#include "inputdev_autoc.h"

T_TX_InterfaceAUTOC::T_TX_InterfaceAUTOC()
{
#if DEBUG_TX_INTERFACE > 0
  printf("T_TX_InterfaceAUTOC::T_TX_InterfaceAUTOC()\n");
#endif
}

T_TX_InterfaceAUTOC::~T_TX_InterfaceAUTOC()
{
#if DEBUG_TX_INTERFACE > 0
  printf("T_TX_InterfaceAUTOC::~T_TX_InterfaceAUTOC()\n");
#endif
}

int T_TX_InterfaceAUTOC::init(SimpleXMLTransfer *config)
{
#if DEBUG_TX_INTERFACE > 0
  printf("int T_TX_InterfaceAUTOC::init(SimpleXMLTransfer* config)\n");
#endif
  T_TX_Interface::init(config);

  boost::asio::io_context io_context;
  socket_ = new tcp::socket(io_context);
  tcp::resolver resolver(io_context);
  auto endpoints = resolver.resolve("localhost", std::to_string(cfg->callback_port));
  boost::asio::connect(*socket_, endpoints);
  return (0);
}

void T_TX_InterfaceAUTOC::putBackIntoCfg(SimpleXMLTransfer *config)
{
#if DEBUG_TX_INTERFACE > 0
  printf("int T_TX_InterfaceAUTOC::putBackIntoCfg(SimpleXMLTransfer* config)\n");
#endif
}

void T_TX_InterfaceAUTOC::getInputData(TSimInputs *inputs)
{
#if DEBUG_TX_INTERFACE > 1
  printf("void T_TX_InterfaceAUTOC::getInputData(TSimInputs* inputs)\n");
#endif
  // occasionally ask for a model update?
  unsigned long simTimeMsec = Global::Simulation->getGameTimeSinceReset();
  if (simTimeMsec > lastUpdateTimeMsec + INPUT_UPDATE_INTERVAL_MSEC)
  {
    lastUpdateTimeMsec = simTimeMsec;

    // convert crrcsim state to AircraftState
    double v = Global::aircraft->getFDM()->getVRelAirmass() * FEET_TO_METERS;
    if (isnan(v) || isinf(v))
    {
      v = 0.0;
    }

    // Create a rotation object from Euler angles
    Eigen::AngleAxisd rollAngle(Global::aircraft->getFDM()->getPhi(), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(Global::aircraft->getFDM()->getTheta(), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(Global::aircraft->getFDM()->getPsi(), Eigen::Vector3d::UnitZ());

    // Combine rotations
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

    // position
    Eigen::Vector3d p{(Global::aircraft->getPos().r[0] - offsetX) * FEET_TO_METERS,
                      (Global::aircraft->getPos().r[1] - offsetY) * FEET_TO_METERS,
                      (Global::aircraft->getPos().r[2] - offsetZ) * FEET_TO_METERS};
    if (isnan(p[0]) || isnan(p[1]) || isnan(p[2]))
    {
      p = Eigen::Vector3d::Zero();
    }

    bool simCrashed = (Global::Simulation->getState() == STATE_CRASHED);

    AircraftState aircraftState{v, q, p,
                                pitchCommand, rollCommand, throttleCommand,
                                simTimeMsec, simCrashed};

    // always send our state
    sendRPC(*socket_, aircraftState);

    // ok what does main say to do
    MainToSim mainToSim = receiveRPC<MainToSim>(*socket_);

    switch (mainToSim.controlType)
    {
    // here we get a reset from the main controller, just update local state (reset, etc)
    case ControlType::AIRCRAFT_STATE:
      // re-initialize aircraft state from incoming payload
      // cfgfile->setAttributeOverwrite("launch.velocity_rel", std::to_string(mainToSim.aircraftState.dRelVel / FEET_TO_METERS));
      // cfgfile->setAttributeOverwrite("launch.altitude", std::to_string(-mainToSim.aircraftState.position[2] / FEET_TO_METERS));
      // cfgfile->setAttributeOverwrite("launch.angle", 0.0); // TODO we need real orientation applied

      // aircraft.setPitchCommand(mainToSim.aircraftState.pitchCommand);
      // aircraft.setRollCommand(mainToSim.aircraftState.rollCommand);
      // aircraft.setThrottleCommand(mainToSim.aircraftState.throttleCommand);

      // cfg->getCurLocCfgPtr(cfgfile)->setAttributeOverwrite("start.position", "");
      // cfg->getCurLocCfgPtr(cfgfile)->setAttributeOverwrite("launch.rel_front", "0.0");
      // cfg->getCurLocCfgPtr(cfgfile)->setAttributeOverwrite("launch.rel_right", "0.0");

      // aircraft.aircraft_orientation = mainToSim.aircraftState.aircraft_orientation;
      simTimeMsec = lastUpdateTimeMsec = 0;
      simCrashed = false;

      // reset sim
      Global::Simulation->reset();

      // extract the initial position offsets in crrcsim coordinate body
      offsetX = 0; // Global::aircraft->getPos().r[0];
      offsetY = 0; // Global::aircraft->getPos().r[1];
      offsetZ = 0; // Global::aircraft->getPos().r[2];

      // char outbuf[1000];
      // sprintf(outbuf, "reset: %010ld % 8.2f %8.2f %8.2f\n", simTimeMsec, offsetX, offsetY, offsetZ);
      // cout << outbuf;

      // reset commands to default
      pitchCommand = 0;
      rollCommand = 0;
      throttleCommand = 0;

      break;

    // here we receive some control signals, simulate
    case ControlType::CONTROL_SIGNAL:
      // update controls

      // cache control commands
      pitchCommand = mainToSim.controlSignal.pitchCommand;
      rollCommand = mainToSim.controlSignal.rollCommand;
      throttleCommand = mainToSim.controlSignal.throttleCommand;

      break;
    }

    // convert cached values to crrcsim scales and return
    inputs->elevator = -pitchCommand / 2.0;          // from -1:1 to -0.5:0.5
    inputs->aileron = rollCommand / 2.0;            // from -1:1 to -0.5:0.5
    inputs->throttle = throttleCommand / 2.0 + 0.5; // from -1:1 to 0:1
  }
}
