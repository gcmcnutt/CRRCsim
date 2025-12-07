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
#include "../../mod_misc/crrc_rand.h"
#include "inputdev_autoc.h"
#include <chrono>
#include <stdio.h>
#include <boost/archive/binary_iarchive.hpp>
#include <cstdlib>

using namespace std::chrono;
using namespace std;
using boost::asio::ip::tcp;

namespace {

void ensureScenarioMetadata(EvalData& evalData) {
  if (evalData.scenarioList.size() == evalData.pathList.size()) {
    return;
  }
  evalData.scenarioList.assign(evalData.pathList.size(), evalData.scenario);
  for (size_t idx = 0; idx < evalData.scenarioList.size(); ++idx) {
    if (evalData.scenarioList[idx].pathVariantIndex < 0) {
      evalData.scenarioList[idx].pathVariantIndex = static_cast<int>(idx);
    }
  }
}

ScenarioMetadata scenarioForPathIndex(const EvalData& evalData, size_t idx) {
  if (idx < evalData.scenarioList.size()) {
    return evalData.scenarioList.at(idx);
  }
  ScenarioMetadata meta = evalData.scenario;
  if (meta.pathVariantIndex < 0) {
    meta.pathVariantIndex = static_cast<int>(idx);
  }
  return meta;
}

} // namespace

static bool gAutocDeterministicMode = (std::getenv("AUTOC_DETERMINISTIC") != nullptr);

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
      ensureScenarioMetadata(evalData);
      
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
      if (!evalData.scenarioList.empty()) {
        evalResults.scenario = evalData.scenarioList.front();
      } else {
        evalResults.scenario = evalData.scenario;
      }
      evalResults.scenarioList.clear();
      evalResults.scenarioList.reserve(evalData.scenarioList.size());

#ifdef DETAILED_LOGGING
      if (gAutocDeterministicMode && !evalData.scenarioList.empty()) {
        const auto& firstScenario = evalData.scenarioList.front();
        std::cerr << "AUTOC deterministic wind seed=" << firstScenario.windSeed
                  << " pathVariant=" << firstScenario.pathVariantIndex
                  << " windVariant=" << firstScenario.windVariantIndex << std::endl;
      }
#endif
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
      ScenarioMetadata activeScenario = scenarioForPathIndex(evalData, static_cast<size_t>(pathSelector));

      // reset sim
      Global::Simulation->reset();
      simCrashed = false;
      lastUpdateTimeMsec = 0;
      pathIndex = 0;
      aircraftStates.clear();

      CRRC_Random::reset(activeScenario.windSeed);
      Init_mod_windfield();
#ifdef DETAILED_LOGGING
      if (gAutocDeterministicMode) {
        std::cerr << "AUTOC deterministic path start pathSelector=" << pathSelector
                  << " pathVariant=" << activeScenario.pathVariantIndex
                  << " windVariant=" << activeScenario.windVariantIndex
                  << " windSeed=" << activeScenario.windSeed << std::endl;
      }
#endif
      
      // Record initial aircraft state at time 0 to match path start
      // Get initial position and orientation after reset
      gp_vec3 initialPos{static_cast<gp_scalar>(Global::aircraft->getPos().r[0] * FEET_TO_METERS),
                                static_cast<gp_scalar>(Global::aircraft->getPos().r[1] * FEET_TO_METERS),
                                static_cast<gp_scalar>(Global::aircraft->getPos().r[2] * FEET_TO_METERS)};
      
      EOM01* eom01 = dynamic_cast<EOM01*>(Global::aircraft->getFDM());
      gp_quat initialQuat;
      if (eom01) {
        initialQuat = gp_quat(eom01->getQuatW(), eom01->getQuatX(), eom01->getQuatY(), eom01->getQuatZ());
      } else {
        initialQuat = gp_quat::Identity();
      }
      initialQuat.normalize();
      
      CRRCMath::Vector3 fdm_velocity = Global::aircraft->getFDM()->getVel();
      gp_vec3 initialVel{
          static_cast<gp_scalar>(fdm_velocity.r[0] * FEET_TO_METERS),
          static_cast<gp_scalar>(fdm_velocity.r[1] * FEET_TO_METERS), 
          static_cast<gp_scalar>(fdm_velocity.r[2] * FEET_TO_METERS)
      };
      gp_scalar initialSpeed = initialVel.norm();
      
      AircraftState initialState{0, initialSpeed, initialVel, initialQuat, initialPos,
                                 static_cast<gp_scalar>(0.0f), static_cast<gp_scalar>(0.0f), static_cast<gp_scalar>(0.0f), 0};
      aircraftStates.push_back(initialState);

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
    gp_vec3 velocity_vector{
        fdm_velocity.r[0] * FEET_TO_METERS,  // North
        fdm_velocity.r[1] * FEET_TO_METERS,  // East
        fdm_velocity.r[2] * FEET_TO_METERS   // Down
    };
    if (isnan(velocity_vector[0]) || isnan(velocity_vector[1]) || isnan(velocity_vector[2]))
    {
      velocity_vector = gp_vec3::Zero();
    }

    // compute ground speed magnitude (consistent with velocity vector)
    gp_scalar v = velocity_vector.norm();
    if (isnan(v) || isinf(v))
    {
      v = 0.0f;
    }

    // Access native quaternion from EOM01 FDM instead of reconstructing from Euler angles
    gp_quat q;
    
    // Try to cast to EOM01 to access native quaternion components
    EOM01* eom01 = dynamic_cast<EOM01*>(Global::aircraft->getFDM());
    if (eom01) {
      // Use native quaternion components from EOM01 (w, x, y, z format)
      q = gp_quat(eom01->getQuatW(), eom01->getQuatX(), eom01->getQuatY(), eom01->getQuatZ());
      
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
    gp_vec3 p{static_cast<gp_scalar>(Global::aircraft->getPos().r[0] * FEET_TO_METERS),
              static_cast<gp_scalar>(Global::aircraft->getPos().r[1] * FEET_TO_METERS),
              static_cast<gp_scalar>(Global::aircraft->getPos().r[2] * FEET_TO_METERS)};
    if (isnan(p[0]) || isnan(p[1]) || isnan(p[2]))
    {
      p = gp_vec3::Zero();
    }

    // search for location of next timestamp using time-based targeting
    while (pathIndex < path.size() - 2 && (path.at(pathIndex).simTimeMsec < simTimeMsec))
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
      printf("%s: aircraft_conversion: crrcsim_pos_ft[%8.2f,%8.2f,%8.2f] -> autoc_pos_m[%8.2f,%8.2f,%8.2f] v_ft=%8.2f->v_m=%8.2f\n",
             get_iso8601_timestamp(tbuf, sizeof(tbuf)),
             Global::aircraft->getPos().r[0], Global::aircraft->getPos().r[1], Global::aircraft->getPos().r[2],
             p[0], p[1], p[2],
             Global::aircraft->getFDM()->getVRelAirmass(), v);
    }
#endif

    CrashReason crashReason = CrashReason::None;

    // out of bounds?
    gp_scalar distanceFromOrigin = std::sqrt(aircraftState.getPosition()[0] * aircraftState.getPosition()[0] +
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
      ScenarioMetadata pathMeta = scenarioForPathIndex(evalData, static_cast<size_t>(pathSelector));
      evalResults.scenarioList.push_back(pathMeta);

      std::vector<AircraftState> aircraftStatesCopy = aircraftStates;
      evalResults.aircraftStateList.push_back(aircraftStatesCopy);
      aircraftStates.clear();

      // prepare for the next path if any
      if (++pathSelector < evalData.pathList.size())
      {
        path = evalData.pathList.at(pathSelector);
#ifdef DETAILED_LOGGING
        if (gAutocDeterministicMode) {
          ScenarioMetadata nextScenario = scenarioForPathIndex(evalData, static_cast<size_t>(pathSelector));
          std::cerr << "AUTOC deterministic path start pathSelector=" << pathSelector
                    << " pathVariant=" << nextScenario.pathVariantIndex
                    << " windVariant=" << nextScenario.windVariantIndex
                    << " windSeed=" << nextScenario.windSeed << std::endl;
        }
#endif
      }
      else
      {
        // send the results back
#ifdef DETAILED_LOGGING
        evalResults.dump(std::cout);
#endif
        sendRPC(*socket_, evalResults);
        evalDataEmpty = true;
        evalResults.pathList.clear();
        evalResults.aircraftStateList.clear();
        evalResults.crashReasonList.clear();
        evalResults.scenarioList.clear();
      }
      return;
    }

    // BASELINE CONTROLLER DISABLED FOR GP-ONLY LEARNING TEST
    // approximate pitch/roll/throttle to achieve goal

    // *** ROLL: Calculate the vector from craft to target in world frame
    /* COMMENTED OUT - BASELINE CONTROLLER DISABLED
    gp_vec3 craftToTarget = path.at(aircraftState.getThisPathIndex()).start - aircraftState.getPosition();

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
    gp_vec3 target_local = aircraftState.getOrientation().inverse() * craftToTarget;

#ifdef DETAILED_LOGGING
    {
      char tbuf[1000];
      Eigen::Matrix<gp_scalar,3,3> rotMatrix = aircraftState.getOrientation().toRotationMatrix();
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
    gp_vec3 projectedVector(0, target_local.y(), target_local.z());

    // Calculate the angle between the projected vector and the body Z-axis
    gp_scalar rollEstimate = std::atan2(projectedVector.y(), -projectedVector.z());

    // *** PITCH: Calculate the vector from craft to target in world frame if it did rotate
    gp_quat rollRotation(Eigen::AngleAxis<gp_scalar>(rollEstimate, gp_vec3::UnitX()));
    gp_quat virtualOrientation = aircraftState.getOrientation() * rollRotation;

    // Transform target vector to new virtual orientation
    gp_vec3 newLocalTargetVector = virtualOrientation.inverse() * craftToTarget;

    // Calculate pitch angle
    gp_scalar pitchEstimate = std::atan2(-newLocalTargetVector.z(), newLocalTargetVector.x());

    // // now try to determine if pitch up or pitch down makes more sense
    // if (std::abs(pitchEstimate) > M_PI / 2) {
    //   pitchEstimate = (pitchEstimate > 0) ? pitchEstimate - M_PI : pitchEstimate + M_PI;
    //   rollEstimate = -rollEstimate;
    // }

    // range is -1:1 
    // Keep baseline estimates identical to minisim - no experimental changes
    gp_scalar rollCmd = std::clamp(rollEstimate / static_cast<gp_scalar>(M_PI), static_cast<gp_scalar>(-1.0f), static_cast<gp_scalar>(1.0f));    // Clamp to prevent extreme values
    gp_scalar pitchCmd = std::clamp(pitchEstimate / static_cast<gp_scalar>(M_PI), static_cast<gp_scalar>(-1.0f), static_cast<gp_scalar>(1.0f));  // Clamp to prevent extreme values
    aircraftState.setRollCommand(rollCmd);
    aircraftState.setPitchCommand(pitchCmd);
    END COMMENTED OUT SECTION */

    // Controls persist from previous timestep - GP will make incremental adjustments

#ifdef DETAILED_LOGGING
    // BASELINE CONTROLLER LOGGING ALSO DISABLED
    /* COMMENTED OUT - BASELINE CONTROLLER VARIABLES UNDEFINED
    // Store baseline controller estimates for comparison
    gp_scalar baselineRoll = rollCmd;
    gp_scalar baselinePitch = pitchCmd;
    
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
    END COMMENTED OUT SECTION */
#endif

    // THROTTLE BASELINE CONTROLLER ALSO DISABLED
    /* COMMENTED OUT - BASELINE CONTROLLER DISABLED
    // Throttle estimate range is -1:1
    {
      gp_scalar distance = (path.at(aircraftState.getThisPathIndex()).start - aircraftState.getPosition()).norm();
      gp_scalar throttleEstimate = std::clamp((distance - static_cast<gp_scalar>(10.0f)) / aircraftState.getRelVel(), static_cast<gp_scalar>(-1.0f), static_cast<gp_scalar>(1.0f));
      aircraftState.setThrottleCommand(throttleEstimate);
    }
    END COMMENTED OUT SECTION */

#ifdef DETAILED_LOGGING
    {
      char tbuf[100];
      printf("%s: state: v:%f posX:%f posY:%f posZ:%f pCmd:%f rCmd:%f tCmd:%f %ld %d\n",
             get_iso8601_timestamp(tbuf, sizeof(tbuf)),
             aircraftState.getRelVel(), aircraftState.getPosition()[0], aircraftState.getPosition()[1], aircraftState.getPosition()[2],
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

#ifdef DETAILED_LOGGING
    // Log key sensor values that GP can access for diagnostics
    {
      char tbuf[100];
      // Calculate some key sensor values the GP would see
      gp_vec3 velocity_body = aircraftState.getOrientation().inverse() * aircraftState.getVelocity();
      gp_scalar alpha = std::atan2(-velocity_body.z(), velocity_body.x()); // GETALPHA
      gp_scalar beta = std::atan2(velocity_body.y(), velocity_body.x());   // GETBETA
      gp_scalar vel = aircraftState.getRelVel();                          // GETVEL
      gp_scalar dhome = (gp_vec3(0, 0, SIM_INITIAL_ALTITUDE) - aircraftState.getPosition()).norm(); // GETDHOME
      
      // Calculate roll and pitch angles from quaternion for logging
      gp_vec3 euler = aircraftState.getOrientation().toRotationMatrix().eulerAngles(2, 1, 0);
      gp_scalar roll_rad = euler[2];   // Roll angle (rotation around X-axis)
      gp_scalar pitch_rad = euler[1];  // Pitch angle (rotation around Y-axis)
      
      printf("%s: gp_sensors: vel=%8.2f alpha_deg=%8.2f beta_deg=%8.2f dhome=%8.2f velx=%8.2f vely=%8.2f velz=%8.2f roll_rad=%8.4f pitch_rad=%8.4f\n",
             get_iso8601_timestamp(tbuf, sizeof(tbuf)),
             vel, alpha * 180.0/M_PI, beta * 180.0/M_PI, dhome,
             aircraftState.getVelocity().x(), aircraftState.getVelocity().y(), aircraftState.getVelocity().z(),
             roll_rad, pitch_rad);
    }
#endif

    // capture the computed outputs
    pitchCommand = aircraftState.getPitchCommand();
    rollCommand = aircraftState.getRollCommand();
    throttleCommand = aircraftState.getThrottleCommand();

#ifdef DETAILED_LOGGING
    // BASELINE COMPARISON LOGGING ALSO DISABLED
    /* COMMENTED OUT - BASELINE CONTROLLER VARIABLES UNDEFINED  
    // Log the baseline vs GP-modified control commands
    {
      char tbuf[100];
      printf("%s: control_comparison: baseline[roll=%8.2f pitch=%8.2f] -> gp_output[roll=%8.2f pitch=%8.2f throttle=%8.2f]\n",
             get_iso8601_timestamp(tbuf, sizeof(tbuf)),
             baselineRoll, baselinePitch,
             rollCommand, pitchCommand, throttleCommand);
    }
    END COMMENTED OUT SECTION */
#endif

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
