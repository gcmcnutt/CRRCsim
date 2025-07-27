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
#include "../../mod_fdm/eom01/eom01.h"
#include "inputdev_autoc.h"
#include <chrono>
#include <stdio.h>
#include <boost/archive/binary_iarchive.hpp>

using namespace std::chrono;
using namespace std;
using boost::asio::ip::tcp;

// Reference to the global aircraftState used by GP evaluation (from autoc-eval.cc)
extern AircraftState aircraftState;

void MyGP::evaluate() {}
void MyGP::evalTask(WorkerContext& context) {}

boost::iostreams::stream<boost::iostreams::array_source> charArrayToIstream(const std::vector<char> &charArray)
{
  return boost::iostreams::stream<boost::iostreams::array_source>(
      boost::iostreams::array_source(charArray.data(), charArray.size()));
}

#ifdef DETAILED_LOGGING
char *get_iso8601_timestamp(char *buf, size_t len)
{
  auto now = system_clock::now();

  // Get the time since the epoch
  auto now_ms = duration_cast<milliseconds>(now.time_since_epoch());

  // Extract seconds and milliseconds
  auto sec = duration_cast<seconds>(now_ms);
  auto ms = now_ms - sec;

  auto itt = std::chrono::system_clock::to_time_t(now);

  std::strftime(buf, len, "%FT%T", std::gmtime(&itt));

  char ms_str[5]; // ".mmm"
  std::snprintf(ms_str, sizeof(ms_str), ".%03ld", ms.count());

  std::strcat(buf, ms_str);
  std::strcat(buf, "Z");

  return buf;
}
#endif

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
  // Clean up GP and bytecode interpreter
  if (gp)
  {
    delete gp;
    gp = nullptr;
  }
  if (interpreter)
  {
    delete interpreter;
    interpreter = nullptr;
  }
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
  unsigned long simTimeMsec = Global::Simulation->getSimulationTimeSinceReset();

  buffer.push_back(simTimeMsec);
  if (simTimeMsec > lastUpdateTimeMsec + INPUT_UPDATE_INTERVAL_MSEC || ++cycleCounter > CYCLE_COUNTER_OVERFLOW)
  {
#ifdef DETAILED_LOGGING
    {
      char tbuf[100];
      printf("%s: updater: cycleCount[%ld] simState[%d] simTimeMsec[%ld] lastUpdateTimeMsec[%ld]\n",
             get_iso8601_timestamp(tbuf, sizeof(tbuf)), cycleCounter, Global::Simulation->getState(), simTimeMsec, lastUpdateTimeMsec);

      // dump out the buffer time steps
      printf("  buffer: ");
      unsigned long last = buffer[0];
      for (auto &t : buffer)
      {
        printf("%ld(%ld) ", t, t - last);
        last = t;
      }
      printf("\n");
    }
#endif

    // ok, if we have seen some large jump in simTimeMsec (computer pause, etc), let's restart this run
    if (simTimeMsec > lastUpdateTimeMsec + SIM_MAX_INTERVAL_MSEC) {
#ifdef DETAILED_LOGGING
      {
        char tbuf[100];
        printf("%s: simTimeJump: %ld  resetting...\n", get_iso8601_timestamp(tbuf, sizeof(tbuf)), simTimeMsec);
      }
#endif

      priorPathSelector = -1;
      lastUpdateTimeMsec = simTimeMsec;
      cycleCounter = 0;
      return;
    }

    lastUpdateTimeMsec = simTimeMsec;
    cycleCounter = 0;

    // initialize the GP once
    if (!gpInitialized)
    {
      initializeSimGP();
      gpInitialized = true;
    }

    // reload from the GP code?
    if (evalDataEmpty)
    {
      evalData = receiveRPC<EvalData>(*socket_);
      
      Global::Simulation->reset();
      lastUpdateTimeMsec = 0;
      evalDataEmpty = false;
      priorPathSelector = -1;
      pathSelector = 0;
      path = evalData.pathList.at(pathSelector);
      evalResults.aircraftStateList.clear();
      evalResults.crashReasonList.clear();
      evalResults.pathList = evalData.pathList;
      evalResults.gp = evalData.gp;
      aircraftStates.clear();

      // Clean up previous interpreters
      if (gp)
      {
        delete gp;
        gp = nullptr;
      }
      if (interpreter)
      {
        delete interpreter;
        interpreter = nullptr;
      }
      
      // Reset data type flags
      isGPTreeData = false;
      isBytecodeData = false;
      
      // Detect if we have GP tree data or bytecode data
      bool looksLikeBinaryArchive = false;
      if (evalData.gp.size() >= 4) {
        // Check for binary archive magic bytes (boost binary archive typically starts with specific patterns)
        unsigned char firstBytes[4] = {
          (unsigned char)evalData.gp[0], 
          (unsigned char)evalData.gp[1], 
          (unsigned char)evalData.gp[2], 
          (unsigned char)evalData.gp[3]
        };
        // Binary archives often start with version info in binary format
        looksLikeBinaryArchive = (firstBytes[0] >= 0x16 && firstBytes[0] <= 0x20) && 
                                (firstBytes[1] == 0x00 || firstBytes[2] == 0x00);
      }
      
      if (looksLikeBinaryArchive) {
        // This looks like Boost binary serialized bytecode data
        try {
          boost::iostreams::stream<boost::iostreams::array_source> bytecodeStream = charArrayToIstream(evalData.gp);
          boost::archive::binary_iarchive archive(bytecodeStream);
          interpreter = new GPBytecodeInterpreter();
          archive >> *interpreter;
          isBytecodeData = true;
        } catch (const std::exception& e) {
          std::cerr << "Error loading bytecode data: " << e.what() << std::endl;
          return;
        }
      } else {
        // This looks like GP tree data
        try {
          boost::iostreams::stream<boost::iostreams::array_source> gpStream = charArrayToIstream(evalData.gp);
          gp = new MyGP();
          gp->load(gpStream);
          gp->resolveNodeValues(adfNs);
          isGPTreeData = true;
        } catch (const std::exception& e) {
          std::cerr << "Error loading GP tree data: " << e.what() << std::endl;
          return;
        }
      }
      return;
    }

    // ok, if we are on to a new path, reset the simulator
    if (priorPathSelector != pathSelector)
    {
      priorPathSelector = pathSelector;

      // reset sim
      Global::Simulation->reset();
      simCrashed = false;
      lastUpdateTimeMsec = 0;
      pathIndex = 0;
      aircraftStates.clear();

#ifdef DETAILED_LOGGING
      {
        char tbuf[100];
        printf("%s: reset: %ld % 8.2f %8.2f %8.2f\n", get_iso8601_timestamp(tbuf, sizeof(tbuf)),
               simTimeMsec, Global::aircraft->getPos().r[0],
               Global::aircraft->getPos().r[1], Global::aircraft->getPos().r[2]);
      }
#endif

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

      // reset commands to default
      inputs->pitch = pitchCommand = 0;
      inputs->aileron = rollCommand = 0;
      inputs->throttle = throttleCommand = 0;
      return;
    }

    // get actual velocity vector from FDM (in feet/s, convert to m/s) - ground speed
    CRRCMath::Vector3 fdm_velocity = Global::aircraft->getFDM()->getVel();
    Eigen::Vector3d velocity_vector{
        fdm_velocity.r[0] * FEET_TO_METERS,  // North
        fdm_velocity.r[1] * FEET_TO_METERS,  // East
        fdm_velocity.r[2] * FEET_TO_METERS   // Down
    };
    if (isnan(velocity_vector[0]) || isnan(velocity_vector[1]) || isnan(velocity_vector[2]))
    {
      velocity_vector = Eigen::Vector3d::Zero();
    }

    // compute ground speed magnitude (consistent with velocity vector)
    double v = velocity_vector.norm();
    if (isnan(v) || isinf(v))
    {
      v = 0.0;
    }

    // Access native quaternion from EOM01 FDM instead of reconstructing from Euler angles
    Eigen::Quaterniond q;
    
    // Try to cast to EOM01 to access native quaternion components
    EOM01* eom01 = dynamic_cast<EOM01*>(Global::aircraft->getFDM());
    if (eom01) {
      // Use native quaternion components from EOM01 (w, x, y, z format)
      q = Eigen::Quaterniond(eom01->getQuatW(), eom01->getQuatX(), eom01->getQuatY(), eom01->getQuatZ());
      
#ifdef DETAILED_LOGGING
      {
        char tbuf[100];
        printf("%s: quaternion_native: w=%8.4f x=%8.4f y=%8.4f z=%8.4f\n",
               get_iso8601_timestamp(tbuf, sizeof(tbuf)),
               eom01->getQuatW(), eom01->getQuatX(), eom01->getQuatY(), eom01->getQuatZ());
      }
#endif
    } else {
      // ERROR: We must use native quaternion for consistency - no fallback allowed
      std::cerr << "FATAL ERROR: FDM is not EOM01 type - cannot access native quaternion!" << std::endl;
      std::cerr << "This will cause quaternion inconsistency between simulators." << std::endl;
      exit(1);
    }
    q.normalize();

    // position
    Eigen::Vector3d p{Global::aircraft->getPos().r[0] * FEET_TO_METERS,
                      Global::aircraft->getPos().r[1] * FEET_TO_METERS,
                      Global::aircraft->getPos().r[2] * FEET_TO_METERS};
    if (isnan(p[0]) || isnan(p[1]) || isnan(p[2]))
    {
      p = Eigen::Vector3d::Zero();
    }

    // search for location of next timestamp
    double timeDistance = SIM_RABBIT_VELOCITY * simTimeMsec / 1000.0;
    while (pathIndex < path.size() - 2 && (path.at(pathIndex).distanceFromStart < timeDistance))
    {
      pathIndex++;
    }


    // convert sim state to AircraftState
    aircraftState = {pathIndex, v, velocity_vector, q, p,
                     pitchCommand, rollCommand, throttleCommand,
                     simTimeMsec};

#ifdef DETAILED_LOGGING
    {
      char tbuf[1000];
      printf("%s: aircraft_conversion: crrcsim_pos_ft[%8.2f,%8.2f,%8.2f] -> autoc_pos_m[%8.2f,%8.2f,%8.2f] crrcsim_euler[%8.2f,%8.2f,%8.2f] v_ft=%8.2f->v_m=%8.2f\n",
             get_iso8601_timestamp(tbuf, sizeof(tbuf)),
             Global::aircraft->getPos().r[0], Global::aircraft->getPos().r[1], Global::aircraft->getPos().r[2],
             p[0], p[1], p[2],
             rollAngle.angle() * 180.0/M_PI, pitchAngle.angle() * 180.0/M_PI, yawAngle.angle() * 180.0/M_PI,
             Global::aircraft->getFDM()->getVRelAirmass(), v);
    }
#endif

    CrashReason crashReason = CrashReason::None;

    // out of bounds?
    double distanceFromOrigin = std::sqrt(aircraftState.getPosition()[0] * aircraftState.getPosition()[0] +
                                          aircraftState.getPosition()[1] * aircraftState.getPosition()[1]);
    if (aircraftState.getPosition()[2] < SIM_MAX_ELEVATION || // too high
        aircraftState.getPosition()[2] > SIM_MIN_ELEVATION || // too low
        distanceFromOrigin > SIM_PATH_RADIUS_LIMIT)
    { // too far
      crashReason = CrashReason::Eval;
    }

    // sim crashed?
    if (Global::Simulation->getState() == STATE_CRASHED)
    {
      crashReason = CrashReason::Sim;
    }

    if (simTimeMsec > SIM_TOTAL_TIME_MSEC)
    {
      crashReason = CrashReason::Time;
    }

    if (pathIndex >= path.size() - 3)
    {
      crashReason = CrashReason::Distance;
    }

    // crashed or out of time or off the end of the list
    if (crashReason != CrashReason::None)
    {
#ifdef DETAILED_LOGGING
      std::cout << "sim: " << crashReasonToString(crashReason) << " time: " << simTimeMsec << " idx: " << pathIndex << " size: " << path.size() << std::endl;
#endif

      // save the crash state
      evalResults.crashReasonList.push_back(crashReason);

      // save the results list
      std::vector<AircraftState> aircraftStatesCopy = aircraftStates;
      evalResults.aircraftStateList.push_back(aircraftStatesCopy);
      aircraftStates.clear();

      // prepare for the next path if any
      if (++pathSelector < evalData.pathList.size())
      {
        path = evalData.pathList.at(pathSelector);
      }
      else
      {
        // send the results back
#ifdef DETAILED_LOGGING
        evalResults.dump(std::cout);
#endif
        sendRPC(*socket_, evalResults);
        evalDataEmpty = true;
      }
      return;
    }

    // approximate pitch/roll/throttle to achieve goal

    // *** ROLL: Calculate the vector from craft to target in world frame
    Eigen::Vector3d craftToTarget = path.at(aircraftState.getThisPathIndex()).start - aircraftState.getPosition();

#ifdef DETAILED_LOGGING
    {
      char tbuf[1000];
      printf("%s: path_target: pathIdx=%d target[%8.2f,%8.2f,%8.2f] aircraft[%8.2f,%8.2f,%8.2f] vector[%8.2f,%8.2f,%8.2f] dist=%8.2f\n",
             get_iso8601_timestamp(tbuf, sizeof(tbuf)),
             aircraftState.getThisPathIndex(),
             path.at(aircraftState.getThisPathIndex()).start[0],
             path.at(aircraftState.getThisPathIndex()).start[1], 
             path.at(aircraftState.getThisPathIndex()).start[2],
             aircraftState.getPosition()[0], aircraftState.getPosition()[1], aircraftState.getPosition()[2],
             craftToTarget[0], craftToTarget[1], craftToTarget[2],
             craftToTarget.norm());
    }
#endif

    // Transform the craft-to-target vector to body frame
    Eigen::Vector3d target_local = aircraftState.getOrientation().inverse() * craftToTarget;

#ifdef DETAILED_LOGGING
    {
      char tbuf[1000];
      Eigen::Matrix3d rotMatrix = aircraftState.getOrientation().toRotationMatrix();
      printf("%s: coordinate_transform: world_vector[%8.2f,%8.2f,%8.2f] -> body_vector[%8.2f,%8.2f,%8.2f]\n",
             get_iso8601_timestamp(tbuf, sizeof(tbuf)),
             craftToTarget[0], craftToTarget[1], craftToTarget[2],
             target_local[0], target_local[1], target_local[2]);
      printf("%s: rotation_matrix: X[%8.2f,%8.2f,%8.2f] Y[%8.2f,%8.2f,%8.2f] Z[%8.2f,%8.2f,%8.2f]\n",
             get_iso8601_timestamp(tbuf, sizeof(tbuf)),
             rotMatrix(0,0), rotMatrix(0,1), rotMatrix(0,2),
             rotMatrix(1,0), rotMatrix(1,1), rotMatrix(1,2),
             rotMatrix(2,0), rotMatrix(2,1), rotMatrix(2,2));
    }
#endif

    // Project the craft-to-target vector onto the body YZ plane
    Eigen::Vector3d projectedVector(0, target_local.y(), target_local.z());

    // Calculate the angle between the projected vector and the body Z-axis
    double rollEstimate = std::atan2(projectedVector.y(), -projectedVector.z());

    // *** PITCH: Calculate the vector from craft to target in world frame if it did rotate
    Eigen::Quaterniond rollRotation(Eigen::AngleAxisd(rollEstimate, Eigen::Vector3d::UnitX()));
    Eigen::Quaterniond virtualOrientation = aircraftState.getOrientation() * rollRotation;

    // Transform target vector to new virtual orientation
    Eigen::Vector3d newLocalTargetVector = virtualOrientation.inverse() * craftToTarget;

    // Calculate pitch angle
    double pitchEstimate = std::atan2(-newLocalTargetVector.z(), newLocalTargetVector.x());

    // // now try to determine if pitch up or pitch down makes more sense
    // if (std::abs(pitchEstimate) > M_PI / 2) {
    //   pitchEstimate = (pitchEstimate > 0) ? pitchEstimate - M_PI : pitchEstimate + M_PI;
    //   rollEstimate = -rollEstimate;
    // }

    // range is -1:1 
    // Keep baseline estimates identical to minisim - no experimental changes
    double rollCmd = std::clamp(rollEstimate / M_PI, -1.0, 1.0);    // Clamp to prevent extreme values
    double pitchCmd = std::clamp(pitchEstimate / M_PI, -1.0, 1.0);  // Clamp to prevent extreme values
    aircraftState.setRollCommand(rollCmd);
    aircraftState.setPitchCommand(pitchCmd);

#ifdef DETAILED_LOGGING
    {
      char tbuf[1000];
      printf("%s: control_calculation: rollEst_rad=%8.2f pitchEst_rad=%8.2f -> rollCmd=%8.2f pitchCmd=%8.2f\n",
             get_iso8601_timestamp(tbuf, sizeof(tbuf)),
             rollEstimate, pitchEstimate,
             rollEstimate / M_PI, pitchEstimate / M_PI);
      printf("%s: projectedVector[%8.2f,%8.2f,%8.2f] rollEst=atan2(%8.2f,%8.2f)\n",
             get_iso8601_timestamp(tbuf, sizeof(tbuf)),
             projectedVector.x(), projectedVector.y(), projectedVector.z(),
             projectedVector.y(), -projectedVector.z());
    }
#endif

    // Throttle estimate range is -1:1
    {
      double distance = (path.at(aircraftState.getThisPathIndex()).start - aircraftState.getPosition()).norm();
      double throttleEstimate = std::clamp((distance - 10) / aircraftState.getRelVel(), -1.0, 1.0);
      aircraftState.setThrottleCommand(throttleEstimate);
    }

#ifdef DETAILED_LOGGING
    {
      char tbuf[100];
      printf("%s: state: v:%f posX:%f posY:%f posZ:%f rAng:%f pAng:%f yAng:%f pCmd:%f rCmd:%f tCmd:%f %ld %d\n",
             get_iso8601_timestamp(tbuf, sizeof(tbuf)),
             aircraftState.getRelVel(), aircraftState.getPosition()[0], aircraftState.getPosition()[1], aircraftState.getPosition()[2],
             rollAngle.angle(), pitchAngle.angle(), yawAngle.angle(),
             aircraftState.getPitchCommand(), aircraftState.getRollCommand(), aircraftState.getThrottleCommand(),
             simTimeMsec, Global::Simulation->getState());
    }
#endif

    // evaluate the current state using appropriate method
    if (isGPTreeData) {
      // Now using global aircraftState directly - no copying needed!
      gp->NthMyGene(0)->evaluate(path, *gp, 0);
    } else if (isBytecodeData) {
      // Use bytecode interpreter to evaluate and set control commands
      interpreter->evaluate(aircraftState, path, 0.0);
    }

    // capture the computed outputs
    pitchCommand = aircraftState.getPitchCommand();
    rollCommand = aircraftState.getRollCommand();
    throttleCommand = aircraftState.getThrottleCommand();

    // save the state for results
    aircraftStates.push_back(aircraftState);

    // convert cached values to crrcsim scales and return
    // NOTE: Critical fix - GP/bytecode expects different coordinate conventions than crrcsim
    inputs->elevator = -pitchCommand / 2.0;         // invert from -1:1 to -0.5:0.5 (crrcsim convention)
    inputs->aileron = rollCommand / 2.0;            // from -1:1 to -0.5:0.5
    inputs->throttle = throttleCommand / 2.0 + 0.5; // from -1:1 to 0:1
    
#ifdef DETAILED_LOGGING
    {
      char tbuf[1000];
      printf("%s: final_inputs: elevator:%8.2f aileron:%8.2f throttle:%8.2f (from pitch:%8.2f roll:%8.2f throttle:%8.2f)\n", 
             get_iso8601_timestamp(tbuf, sizeof(tbuf)),
             inputs->elevator, inputs->aileron, inputs->throttle,
             pitchCommand, rollCommand, throttleCommand);
    }
#endif
  }
}
