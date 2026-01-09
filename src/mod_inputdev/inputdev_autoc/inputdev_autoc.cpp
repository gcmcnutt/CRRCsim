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
#include "../../mod_windfield/windfield.h"
#include "inputdev_autoc.h"
#include <chrono>
#include <stdio.h>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
#include <cstdlib>
#include <sstream>

using namespace std::chrono;
using namespace std;
using boost::asio::ip::tcp;

namespace {

// Hash serialized data instead of raw memory to avoid padding issues
template<typename T>
std::pair<const void*, size_t> serializeForHash(const T& data, std::vector<char>& buffer) {
  buffer.clear();
  boost::iostreams::back_insert_device<std::vector<char>> inserter(buffer);
  boost::iostreams::stream<boost::iostreams::back_insert_device<std::vector<char>>> stream(inserter);
  boost::archive::binary_oarchive archive(stream);
  archive << data;
  stream.flush();
  return std::make_pair(buffer.data(), buffer.size());
}

// Runtime-configurable intervals (sim time)
unsigned long parseIntervalFromEnv(const char* name, unsigned long fallback) {
  const char* env = std::getenv(name);
  if (!env || *env == '\0') {
    return fallback;
  }
  char* endptr = nullptr;
  unsigned long val = strtoul(env, &endptr, 10);
  if (endptr == env || val == 0) {
    return fallback;
  }
  // clamp to a sane range (5ms .. 1000ms)
  if (val < 5) val = 5;
  if (val > 1000) val = 1000;
  return val;
}

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

// Global intervals used by AUTOC input (declared in header)
unsigned long gEvalUpdateIntervalMsec = EVAL_UPDATE_INTERVAL_MSEC_DEFAULT;   // sensor/GP cadence
unsigned long gComputeLatencyMsec = COMPUTE_LATENCY_MSEC_DEFAULT;           // simulated GP compute latency (sensor→output)

unsigned long getCycleCounterOverflow() {
  // a few cycles after the last update, we assume crash, no flight updates, etc
  unsigned long overflow = static_cast<unsigned long>(SIM_FPS * gEvalUpdateIntervalMsec / 1000.0);
  return overflow > 0 ? overflow : 1;
}

static bool gInDeterministicTest = false;

// Reference to the global aircraftState used by GP evaluation (from autoc-eval.cc)
extern AircraftState aircraftState;

// Thread-local physics trace buffer for collecting detailed FDM state
// Cleared at start of each evaluation, sent back with EvalResults
thread_local std::vector<PhysicsTraceEntry> gCurrentPhysicsTrace;

// Extern globals defined in fdm_larcsim.cpp for trace capture
// We set these so the FDM can access worker identity when capturing traces
extern "C" {
  extern int32_t gTraceWorkerId;
  extern int32_t gTraceWorkerPid;
  extern int32_t gTraceEvalCounter;
  extern int32_t gTracePathIndex;
  extern uint32_t gPhysicsStepCounter;
}

// Helper function to capture current FDM state into physics trace
// Called from FDM code (fdm_larcsim::update) to log detailed physics state
// Declared extern "C" to make it easily callable from FDM
extern "C" void capturePhysicsTrace(
    uint32_t step, double simTimeMsec, double dtSec,
    int32_t workerId, int32_t workerPid, int32_t evalCounter,
    const double* pos, const double* vel, const double* acc, const double* accPast,
    const double* quat, const double* quatDotPast,
    const double* omegaBody, const double* omegaDotBody,
    const double* rate, const double* ratePast,
    double alpha, double beta, double vRelWind,
    const double* velRelGround, const double* velRelAir,
    const double* vLocal, const double* vLocalDot,
    double cosAlpha, double sinAlpha, double cosBeta,
    double CL, double CD,
    double CL_left, double CL_cent, double CL_right, double CL_wing,
    double Cl, double Cm, double Cn, double QS,
    const double* forceBody, const double* momentBody,
    const double* wind, const double* localAirmass, const double* gustBody,
    double density, double gravity,
    double geocentricLat, double geocentricLon, double geocentricR,
    double pitchCommand, double rollCommand, double throttleCommand,
    double elevator, double aileron, double rudder, double throttle,
    uint16_t rngState16, uint32_t rngState32,
    int32_t pathIndex) {

  PhysicsTraceEntry entry;
  entry.step = step;
  entry.simTimeMsec = simTimeMsec;
  entry.dtSec = dtSec;
  entry.workerId = workerId;
  entry.workerPid = workerPid;
  entry.evalCounter = evalCounter;

  memcpy(entry.pos, pos, 3 * sizeof(double));
  memcpy(entry.vel, vel, 3 * sizeof(double));
  memcpy(entry.acc, acc, 3 * sizeof(double));
  memcpy(entry.accPast, accPast, 3 * sizeof(double));
  memcpy(entry.quat, quat, 4 * sizeof(double));
  memcpy(entry.quatDotPast, quatDotPast, 4 * sizeof(double));
  memcpy(entry.omegaBody, omegaBody, 3 * sizeof(double));
  memcpy(entry.omegaDotBody, omegaDotBody, 3 * sizeof(double));
  memcpy(entry.rate, rate, 3 * sizeof(double));
  memcpy(entry.ratePast, ratePast, 3 * sizeof(double));

  entry.alpha = alpha;
  entry.beta = beta;
  entry.vRelWind = vRelWind;
  memcpy(entry.velRelGround, velRelGround, 3 * sizeof(double));
  memcpy(entry.velRelAir, velRelAir, 3 * sizeof(double));
  memcpy(entry.vLocal, vLocal, 3 * sizeof(double));
  memcpy(entry.vLocalDot, vLocalDot, 3 * sizeof(double));

  entry.cosAlpha = cosAlpha;
  entry.sinAlpha = sinAlpha;
  entry.cosBeta = cosBeta;
  entry.CL = CL;
  entry.CD = CD;
  entry.CL_left = CL_left;
  entry.CL_cent = CL_cent;
  entry.CL_right = CL_right;
  entry.CL_wing = CL_wing;
  entry.Cl = Cl;
  entry.Cm = Cm;
  entry.Cn = Cn;
  entry.QS = QS;

  memcpy(entry.forceBody, forceBody, 3 * sizeof(double));
  memcpy(entry.momentBody, momentBody, 3 * sizeof(double));
  memcpy(entry.wind, wind, 3 * sizeof(double));
  memcpy(entry.localAirmass, localAirmass, 3 * sizeof(double));
  memcpy(entry.gustBody, gustBody, 6 * sizeof(double));  // Now 6 elements: v_V_gust + v_R_omega_gust

  entry.density = density;
  entry.gravity = gravity;
  entry.geocentricLat = geocentricLat;
  entry.geocentricLon = geocentricLon;
  entry.geocentricR = geocentricR;

  entry.pitchCommand = pitchCommand;
  entry.rollCommand = rollCommand;
  entry.throttleCommand = throttleCommand;
  entry.elevator = elevator;
  entry.aileron = aileron;
  entry.rudder = rudder;
  entry.throttle = throttle;

  entry.rngState16 = rngState16;
  entry.rngState32 = rngState32;
  entry.pathIndex = pathIndex;

  gCurrentPhysicsTrace.push_back(entry);
}

// Single pending command to model compute latency between sensor sample and applied outputs
struct PendingCommand {
  bool valid = false;
  unsigned long readyTimeMsec = 0;
  gp_scalar pitch = 0;
  gp_scalar roll = 0;
  gp_scalar throttle = 0;
};
static PendingCommand gPendingCommand;

void MyGP::evaluate() {}
void MyGP::evalTask(WorkerContext& context) {}

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
  : callbackPort(0)  // Initialize to safe default
{
#if DEBUG_TX_INTERFACE > 0
  printf("T_TX_InterfaceAUTOC::T_TX_InterfaceAUTOC()\n");
#endif
  workerPid = static_cast<int>(getpid());
  workerIndex = 0;
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
  // Allow runtime override of GP eval cadence and compute latency (sim time).
  gEvalUpdateIntervalMsec = parseIntervalFromEnv("AUTOC_EVAL_INTERVAL_MSEC", EVAL_UPDATE_INTERVAL_MSEC_DEFAULT);
  gComputeLatencyMsec = parseIntervalFromEnv("AUTOC_COMPUTE_LATENCY_MSEC", COMPUTE_LATENCY_MSEC_DEFAULT);
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
  constexpr size_t DEBUG_SAMPLE_LIMIT = 64;
  // occasionally ask for a model update?
  unsigned long simTimeMsec = Global::Simulation->getSimulationTimeSinceReset();

  // Always emit the last cached commands each frame; eval only on cadence
  buffer.push_back(simTimeMsec);
  const unsigned long overflowLimit = getCycleCounterOverflow();
  bool shouldEval = (simTimeMsec > lastUpdateTimeMsec + gEvalUpdateIntervalMsec) || (++cycleCounter > overflowLimit);
  if (shouldEval)
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
      gPendingCommand = PendingCommand{};
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
      const uint64_t localGpHash = hashByteVector(evalData.gp);
      if (evalData.gpHash == 0) {
        evalData.gpHash = localGpHash;
      }
      ensureScenarioMetadata(evalData);

      // Set flag for RNG tracing when entering deterministic test
      gInDeterministicTest = (!evalData.scenarioList.empty() &&
                              evalData.scenarioList.front().enableDeterministicLogging);

      // Enable determinism tracker on first deterministic test
      static bool trackerInitialized = false;
      if (gInDeterministicTest && !trackerInitialized) {
        // Determinism tracker disabled
        trackerInitialized = true;
      }

      // Log job receipt with hash verification
      // (disabled)

      // Log scenario metadata hash during determinism test
      // (disabled)

      evalDataEmpty = false;
      priorPathSelector = -1;
      pathSelector = 0;
      path = evalData.pathList.at(pathSelector);
      gPendingCommand = PendingCommand{};
      evalResults.aircraftStateList.clear();
      evalResults.crashReasonList.clear();
      evalResults.pathList = evalData.pathList;
      evalResults.gp = evalData.gp;
      evalResults.gpHash = evalData.gpHash;
      if (evalResults.gpHash == 0 && !evalResults.gp.empty()) {
        evalResults.gpHash = hashByteVector(evalResults.gp);
      }
      if (!evalData.scenarioList.empty()) {
        evalResults.scenario = evalData.scenarioList.front();
      } else {
        evalResults.scenario = evalData.scenario;
      }
      evalResults.scenarioList.clear();
      evalResults.scenarioList.reserve(evalData.scenarioList.size());
      evalResults.debugSamples.clear();
      evalResults.workerId = workerIndex;
      evalResults.workerPid = workerPid;
      evalResults.workerEvalCounter = evalCounter;

#ifdef DETAILED_LOGGING
      if (!evalData.scenarioList.empty()) {
        const auto& firstScenario = evalData.scenarioList.front();
        std::cerr << "AUTOC deterministic wind seed=" << firstScenario.windSeed
                  << " pathVariant=" << firstScenario.pathVariantIndex
                  << " windVariant=" << firstScenario.windVariantIndex << std::endl;
      }
#endif
      aircraftStates.clear();
      debugSamplesCurrentPath.clear();

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
          boost::iostreams::array_source source(evalData.gp.data(), evalData.gp.size());
          boost::iostreams::stream<boost::iostreams::array_source> bytecodeStream(source);
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
          boost::iostreams::array_source source(evalData.gp.data(), evalData.gp.size());
          boost::iostreams::stream<boost::iostreams::array_source> gpStream(source);
          gp = new MyGP();
          gp->load(gpStream);
          gp->resolveNodeValues(adfNs);
          isGPTreeData = true;
        } catch (const std::exception& e) {
          std::cerr << "Error loading GP tree data: " << e.what() << std::endl;
          return;
        }
      }
      // Cache quat dot past reset
      quatDotPast[0] = quatDotPast[1] = quatDotPast[2] = quatDotPast[3] = 0.0;
      return;
    }

    // ok, if we are on to a new path, reset the simulator
    if (priorPathSelector != pathSelector)
    {
      priorPathSelector = pathSelector;
      ScenarioMetadata activeScenario = scenarioForPathIndex(evalData, static_cast<size_t>(pathSelector));

      // Reset simulation with wind seed
      // This will:
      // 1. Set RNG to windSeed
      // 2. Call Init_mod_windfield() which creates thermals (consuming RNG deterministically)
      // 3. Call initialize_gust() which resets all RandGauss objects to phase=0
      Global::Simulation->reset(activeScenario.windSeed);
      simCrashed = false;
      lastUpdateTimeMsec = 0;
      pathIndex = 0;
      gPendingCommand = PendingCommand{};
      aircraftStates.clear();
      gCurrentPhysicsTrace.clear();  // Clear physics trace for new path

      // Set worker identity globals for FDM trace capture
      gTraceWorkerId = workerIndex;
      gTraceWorkerPid = workerPid;
      gTraceEvalCounter = evalCounter;
      gTracePathIndex = pathSelector;
      gPhysicsStepCounter = 0;  // Reset step counter for new path

      // Start ordered event logging for this path
      RandGaussTrace::startEventLog(1000);
#ifdef DETAILED_LOGGING
      std::cerr << "AUTOC deterministic path start pathSelector=" << pathSelector
                << " pathVariant=" << activeScenario.pathVariantIndex
                << " windVariant=" << activeScenario.windVariantIndex
                << " windSeed=" << activeScenario.windSeed << std::endl;
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
      if (debugSamplesCurrentPath.empty()) {
        DebugSample sample;
        sample.pathIndex = pathSelector;
        sample.stepIndex = 0;
        sample.simTimeMsec = static_cast<gp_scalar>(0.0f);
        sample.dtSec = static_cast<gp_scalar>(Global::dt);
        sample.simSteps = static_cast<gp_scalar>(Global::Simulation->getSimulationTimeSinceReset() / 1000.0);
        sample.velRelGround = initialVel;
        sample.velRelAirmass = gp_vec3::Zero();
        sample.position = initialPos;
        sample.velocity = initialVel;
        sample.acceleration = gp_vec3::Zero();
        sample.accelPast = gp_vec3::Zero();
        sample.angularRates = gp_vec3::Zero();
        sample.angularAccelPast = gp_vec3::Zero();
        sample.quatDotPast = gp_quat::Identity();
        sample.windBody = gp_vec3::Zero();
        sample.orientation = initialQuat;
        sample.pitchCommand = static_cast<gp_scalar>(0.0f);
        sample.rollCommand = static_cast<gp_scalar>(0.0f);
        sample.throttleCommand = static_cast<gp_scalar>(0.0f);
        sample.elevatorSim = static_cast<gp_scalar>(0.0f);
        sample.aileronSim = static_cast<gp_scalar>(0.0f);
        sample.throttleSim = static_cast<gp_scalar>(0.0f);
        sample.massKg = static_cast<gp_scalar>(eom01 ? eom01->getMass() : 0.0f);
        sample.density = static_cast<gp_scalar>(eom01 ? eom01->getDensity() : 0.0f);
        sample.gravity = static_cast<gp_scalar>(eom01 ? eom01->getGravity() : 0.0f);
        sample.alpha = static_cast<gp_scalar>(eom01 ? eom01->getAlpha() : 0.0f);
        sample.beta = static_cast<gp_scalar>(eom01 ? eom01->getBeta() : 0.0f);
        sample.vRelWind = static_cast<gp_scalar>(eom01 ? eom01->getVRelWind() : 0.0f);
        if (eom01) {
          CRRCMath::Vector3 vLocal = eom01->getVLocal();
          sample.vLocal = gp_vec3{
              static_cast<gp_scalar>(vLocal.r[0] * FEET_TO_METERS),
              static_cast<gp_scalar>(vLocal.r[1] * FEET_TO_METERS),
              static_cast<gp_scalar>(vLocal.r[2] * FEET_TO_METERS)};
          CRRCMath::Vector3 vLocalDot = eom01->getVLocalDot();
          sample.vLocalDot = gp_vec3{
              static_cast<gp_scalar>(vLocalDot.r[0] * FEET_TO_METERS),
              static_cast<gp_scalar>(vLocalDot.r[1] * FEET_TO_METERS),
              static_cast<gp_scalar>(vLocalDot.r[2] * FEET_TO_METERS)};
          CRRCMath::Vector3 omegaBody = eom01->getOmegaBody();
          sample.omegaBody = gp_vec3{
              static_cast<gp_scalar>(omegaBody.r[0]),
              static_cast<gp_scalar>(omegaBody.r[1]),
              static_cast<gp_scalar>(omegaBody.r[2])};
          CRRCMath::Vector3 omegaDotBody = eom01->getOmegaDot();
          sample.omegaDotBody = gp_vec3{
              static_cast<gp_scalar>(omegaDotBody.r[0]),
              static_cast<gp_scalar>(omegaDotBody.r[1]),
              static_cast<gp_scalar>(omegaDotBody.r[2])};
          sample.latGeoc = static_cast<gp_scalar>(eom01->getLatGeocentric());
          sample.lonGeoc = static_cast<gp_scalar>(eom01->getLonGeocentric());
          sample.radiusToVehicle = static_cast<gp_scalar>(eom01->getRadiusToVehicle() * FEET_TO_METERS);
        } else {
          sample.vLocal = gp_vec3::Zero();
          sample.vLocalDot = gp_vec3::Zero();
          sample.omegaBody = gp_vec3::Zero();
          sample.omegaDotBody = gp_vec3::Zero();
          sample.latGeoc = static_cast<gp_scalar>(0.0f);
          sample.lonGeoc = static_cast<gp_scalar>(0.0f);
          sample.radiusToVehicle = static_cast<gp_scalar>(0.0f);
        }
        sample.latDotPast = static_cast<gp_scalar>(eom01 ? eom01->getLatDotPast() : 0.0f);
        sample.lonDotPast = static_cast<gp_scalar>(eom01 ? eom01->getLonDotPast() : 0.0f);
        sample.radiusDotPast = static_cast<gp_scalar>(eom01 ? eom01->getRadiusDotPast() : 0.0f);
        sample.rngState16 = CRRC_Random::getState16();
        sample.rngState32 = CRRC_Random::getState32();
        debugSamplesCurrentPath.push_back(sample);
      }

      // Increment evaluation counter BEFORE any logging (needed for warmup check)
      evalCounter++;

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
    // Commands always start at zero before GP evaluation (prevents pollution across re-evals)
    aircraftState = {pathIndex, v, velocity_vector, q, p,
                     static_cast<gp_scalar>(0.0f), static_cast<gp_scalar>(0.0f), static_cast<gp_scalar>(0.0f),
                     simTimeMsec};

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
      if (!debugSamplesCurrentPath.empty()) {
        evalResults.debugSamples.push_back(debugSamplesCurrentPath);
        debugSamplesCurrentPath.clear();
      } else {
        evalResults.debugSamples.emplace_back();
      }

      // Dump ordered RandGauss event log for this path
      RandGaussTrace::dumpAndClearEventLog();

      // Add physics trace for this path
      if (!gCurrentPhysicsTrace.empty()) {
        evalResults.physicsTrace.push_back(gCurrentPhysicsTrace);
        gCurrentPhysicsTrace.clear();
      } else {
        evalResults.physicsTrace.emplace_back();
      }

      // Dtest logging disabled

      // prepare for the next path if any
      if (++pathSelector < evalData.pathList.size())
      {
        path = evalData.pathList.at(pathSelector);
#ifdef DETAILED_LOGGING
        ScenarioMetadata nextScenario = scenarioForPathIndex(evalData, static_cast<size_t>(pathSelector));
        std::cerr << "AUTOC deterministic path start pathSelector=" << pathSelector
                  << " pathVariant=" << nextScenario.pathVariantIndex
                  << " windVariant=" << nextScenario.windVariantIndex
                  << " windSeed=" << nextScenario.windSeed << std::endl;
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

    // Evaluate immediately on this snapshot
    if (isGPTreeData) {
      gp->NthMyGene(0)->evaluate(path, *gp, 0);
    } else if (isBytecodeData) {
      interpreter->evaluate(aircraftState, path, 0.0);
    }

    // Save post-eval state for results
    aircraftStates.push_back(aircraftState);
    if (debugSamplesCurrentPath.size() < DEBUG_SAMPLE_LIMIT) {
      DebugSample sample;
      sample.pathIndex = pathSelector;
      sample.stepIndex = static_cast<int>(aircraftStates.size() - 1);
      sample.simTimeMsec = static_cast<gp_scalar>(simTimeMsec);
      sample.dtSec = static_cast<gp_scalar>(Global::dt);
      sample.simSteps = static_cast<gp_scalar>(Global::Simulation->getSimulationTimeSinceReset() / 1000.0);
      if (eom01) {
        CRRCMath::Vector3 vGround = eom01->getVelRelGroundVec();
        sample.velRelGround = gp_vec3{
            static_cast<gp_scalar>(vGround.r[0] * FEET_TO_METERS),
            static_cast<gp_scalar>(vGround.r[1] * FEET_TO_METERS),
            static_cast<gp_scalar>(vGround.r[2] * FEET_TO_METERS)};
        CRRCMath::Vector3 vAir = eom01->getVelRelAirmassVec();
        sample.velRelAirmass = gp_vec3{
            static_cast<gp_scalar>(vAir.r[0] * FEET_TO_METERS),
            static_cast<gp_scalar>(vAir.r[1] * FEET_TO_METERS),
            static_cast<gp_scalar>(vAir.r[2] * FEET_TO_METERS)};
      } else {
        sample.velRelGround = velocity_vector;
        sample.velRelAirmass = gp_vec3::Zero();
      }
      sample.position = p;
      sample.velocity = velocity_vector;
      if (eom01) {
        CRRCMath::Vector3 accelFdm = eom01->getAccel();
        sample.acceleration = gp_vec3{
            static_cast<gp_scalar>(accelFdm.r[0] * FEET_TO_METERS),
            static_cast<gp_scalar>(accelFdm.r[1] * FEET_TO_METERS),
            static_cast<gp_scalar>(accelFdm.r[2] * FEET_TO_METERS)};
        CRRCMath::Vector3 accelPast = eom01->getAccelPast();
        sample.accelPast = gp_vec3{
            static_cast<gp_scalar>(accelPast.r[0] * FEET_TO_METERS),
            static_cast<gp_scalar>(accelPast.r[1] * FEET_TO_METERS),
            static_cast<gp_scalar>(accelPast.r[2] * FEET_TO_METERS)};
        CRRCMath::Vector3 pqr = eom01->getPQR();
        sample.angularRates = gp_vec3{
            static_cast<gp_scalar>(pqr.r[0]),
            static_cast<gp_scalar>(pqr.r[1]),
            static_cast<gp_scalar>(pqr.r[2])};
        CRRCMath::Vector3 omegaDotPast = eom01->getOmegaDotPast();
        sample.angularAccelPast = gp_vec3{
            static_cast<gp_scalar>(omegaDotPast.r[0]),
            static_cast<gp_scalar>(omegaDotPast.r[1]),
            static_cast<gp_scalar>(omegaDotPast.r[2])};
        double quatDotArr[4]; eom01->getQuatDotPast(quatDotArr);
        sample.quatDotPast = gp_quat(
            static_cast<gp_scalar>(quatDotArr[0]),
            static_cast<gp_scalar>(quatDotArr[1]),
            static_cast<gp_scalar>(quatDotArr[2]),
            static_cast<gp_scalar>(quatDotArr[3]));
        CRRCMath::Vector3 windBodyFdm = eom01->getWindBody();
        sample.windBody = gp_vec3{
            static_cast<gp_scalar>(windBodyFdm.r[0] * FEET_TO_METERS),
            static_cast<gp_scalar>(windBodyFdm.r[1] * FEET_TO_METERS),
            static_cast<gp_scalar>(windBodyFdm.r[2] * FEET_TO_METERS)};
        sample.massKg = static_cast<gp_scalar>(eom01->getMass());
        sample.density = static_cast<gp_scalar>(eom01->getDensity());
        sample.gravity = static_cast<gp_scalar>(eom01->getGravity());
        sample.alpha = static_cast<gp_scalar>(eom01->getAlpha());
        sample.beta = static_cast<gp_scalar>(eom01->getBeta());
        sample.vRelWind = static_cast<gp_scalar>(eom01->getVRelWind());
        CRRCMath::Vector3 vLocal = eom01->getVLocal();
        sample.vLocal = gp_vec3{
            static_cast<gp_scalar>(vLocal.r[0] * FEET_TO_METERS),
            static_cast<gp_scalar>(vLocal.r[1] * FEET_TO_METERS),
            static_cast<gp_scalar>(vLocal.r[2] * FEET_TO_METERS)};
        CRRCMath::Vector3 vLocalDot = eom01->getVLocalDot();
        sample.vLocalDot = gp_vec3{
            static_cast<gp_scalar>(vLocalDot.r[0] * FEET_TO_METERS),
            static_cast<gp_scalar>(vLocalDot.r[1] * FEET_TO_METERS),
            static_cast<gp_scalar>(vLocalDot.r[2] * FEET_TO_METERS)};
        CRRCMath::Vector3 omegaBody = eom01->getOmegaBody();
        sample.omegaBody = gp_vec3{
            static_cast<gp_scalar>(omegaBody.r[0]),
            static_cast<gp_scalar>(omegaBody.r[1]),
            static_cast<gp_scalar>(omegaBody.r[2])};
        CRRCMath::Vector3 omegaDotBody = eom01->getOmegaDot();
        sample.omegaDotBody = gp_vec3{
            static_cast<gp_scalar>(omegaDotBody.r[0]),
            static_cast<gp_scalar>(omegaDotBody.r[1]),
            static_cast<gp_scalar>(omegaDotBody.r[2])};
        sample.latGeoc = static_cast<gp_scalar>(eom01->getLatGeocentric());
        sample.lonGeoc = static_cast<gp_scalar>(eom01->getLonGeocentric());
        sample.radiusToVehicle = static_cast<gp_scalar>(eom01->getRadiusToVehicle() * FEET_TO_METERS);
        CRRCMath::Vector3 localAirmass = eom01->getLastLocalAirmass();
        sample.localAirmass = gp_vec3{
            static_cast<gp_scalar>(localAirmass.r[0] * FEET_TO_METERS),
            static_cast<gp_scalar>(localAirmass.r[1] * FEET_TO_METERS),
            static_cast<gp_scalar>(localAirmass.r[2] * FEET_TO_METERS)};
        CRRCMath::Vector3 gustBody = eom01->getLastGustBody();
        sample.gustBody = gp_vec3{
            static_cast<gp_scalar>(gustBody.r[0] * FEET_TO_METERS),
            static_cast<gp_scalar>(gustBody.r[1] * FEET_TO_METERS),
            static_cast<gp_scalar>(gustBody.r[2] * FEET_TO_METERS)};
        CRRCMath::Vector3 forceBody = eom01->getLastForceBody();
        sample.forceBody = gp_vec3{
            static_cast<gp_scalar>(forceBody.r[0] * FEET_TO_METERS),
            static_cast<gp_scalar>(forceBody.r[1] * FEET_TO_METERS),
            static_cast<gp_scalar>(forceBody.r[2] * FEET_TO_METERS)};
        CRRCMath::Vector3 momentBody = eom01->getLastMomentBody();
        sample.momentBody = gp_vec3{
            static_cast<gp_scalar>(momentBody.r[0]),
            static_cast<gp_scalar>(momentBody.r[1]),
            static_cast<gp_scalar>(momentBody.r[2])};
        sample.latDotPast = static_cast<gp_scalar>(eom01->getLatDotPast());
        sample.lonDotPast = static_cast<gp_scalar>(eom01->getLonDotPast());
        sample.radiusDotPast = static_cast<gp_scalar>(eom01->getRadiusDotPast());
      } else {
        sample.acceleration = gp_vec3::Zero();
        sample.accelPast = gp_vec3::Zero();
        sample.angularRates = gp_vec3::Zero();
        sample.angularAccelPast = gp_vec3::Zero();
        sample.quatDotPast = gp_quat::Identity();
        sample.windBody = gp_vec3::Zero();
        sample.massKg = static_cast<gp_scalar>(0.0f);
        sample.density = static_cast<gp_scalar>(0.0f);
        sample.gravity = static_cast<gp_scalar>(0.0f);
        sample.alpha = static_cast<gp_scalar>(0.0f);
        sample.beta = static_cast<gp_scalar>(0.0f);
        sample.vRelWind = static_cast<gp_scalar>(0.0f);
        sample.localAirmass = gp_vec3::Zero();
        sample.gustBody = gp_vec3::Zero();
        sample.forceBody = gp_vec3::Zero();
        sample.momentBody = gp_vec3::Zero();
        sample.vLocal = gp_vec3::Zero();
        sample.vLocalDot = gp_vec3::Zero();
        sample.omegaBody = gp_vec3::Zero();
        sample.omegaDotBody = gp_vec3::Zero();
        sample.latGeoc = static_cast<gp_scalar>(0.0f);
        sample.lonGeoc = static_cast<gp_scalar>(0.0f);
        sample.radiusToVehicle = static_cast<gp_scalar>(0.0f);
        sample.latDotPast = static_cast<gp_scalar>(0.0f);
        sample.lonDotPast = static_cast<gp_scalar>(0.0f);
        sample.radiusDotPast = static_cast<gp_scalar>(0.0f);
      }
      sample.orientation = q;
      sample.pitchCommand = aircraftState.getPitchCommand();
      sample.rollCommand = aircraftState.getRollCommand();
      sample.throttleCommand = aircraftState.getThrottleCommand();
      sample.elevatorSim = static_cast<gp_scalar>(-pitchCommand / 2.0);
      sample.aileronSim = static_cast<gp_scalar>(rollCommand / 2.0);
      sample.throttleSim = static_cast<gp_scalar>(throttleCommand / 2.0 + 0.5);
      sample.rngState16 = CRRC_Random::getState16();
      sample.rngState32 = CRRC_Random::getState32();
      debugSamplesCurrentPath.push_back(sample);
    }

    // Stage commands to be applied after compute latency
    gPendingCommand.pitch = aircraftState.getPitchCommand();
    gPendingCommand.roll = aircraftState.getRollCommand();
    gPendingCommand.throttle = aircraftState.getThrottleCommand();
    gPendingCommand.readyTimeMsec = simTimeMsec + gComputeLatencyMsec;
    gPendingCommand.valid = true;

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
  }

  // Apply pending commands when latency expires
  if (gPendingCommand.valid && simTimeMsec >= gPendingCommand.readyTimeMsec) {
    pitchCommand = gPendingCommand.pitch;
    rollCommand = gPendingCommand.roll;
    throttleCommand = gPendingCommand.throttle;
    gPendingCommand.valid = false;
  }

  // convert cached values to crrcsim scales and return every frame
  // NOTE: Critical fix - GP/bytecode expects different coordinate conventions than crrcsim
  inputs->elevator = -pitchCommand / 2.0;         // invert from -1:1 to -0.5:0.5 (crrcsim convention)
  inputs->aileron = rollCommand / 2.0;            // from -1:1 to -0.5:0.5
  inputs->throttle = throttleCommand / 2.0 + 0.5; // from -1:1 to 0:1
  
#ifdef DETAILED_LOGGING
  {
    char tbuf[1000];
    printf("%s: final_inputs: elevator:%8.2f aileron:%8.2f throttle:%8.2f (from pitch:%8.2f roll:%8.2f throttle:%8.2f) simTime:%ld eval:%s pending:%s\n", 
           get_iso8601_timestamp(tbuf, sizeof(tbuf)),
           inputs->elevator, inputs->aileron, inputs->throttle,
           pitchCommand, rollCommand, throttleCommand,
           simTimeMsec, shouldEval ? "Y" : "N", gPendingCommand.valid ? "Y" : "N");
  }
#endif
}
