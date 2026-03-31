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
#include "autoc/eval/variation_generator.h"
#include <algorithm>
#include <chrono>
#include <stdio.h>
#include <cstdlib>
#include <sstream>

using namespace std::chrono;
using namespace std;


// DETAILED_LOGGING controls noisy RNG trace output - leave undefined for normal use

namespace {

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
unsigned long gEvalUpdateIntervalMsec = EVAL_UPDATE_INTERVAL_MSEC_DEFAULT;   // sensor/NN cadence
unsigned long gComputeLatencyMsec = COMPUTE_LATENCY_MSEC_DEFAULT;           // simulated NN compute latency (sensor→output)

// ACRO mode rate PID state
static double gAcroIntegralRoll = 0.0;
static double gAcroIntegralPitch = 0.0;
static double gAcroIntegralYaw = 0.0;
static unsigned long gAcroLastTimeMsec = 0;

unsigned long getCycleCounterOverflow() {
  // a few cycles after the last update, we assume crash, no flight updates, etc
  unsigned long overflow = static_cast<unsigned long>(SIM_FPS * gEvalUpdateIntervalMsec / 1000.0);
  return overflow > 0 ? overflow : 1;
}

static bool gInDeterministicTest = false;

// Global aircraftState used by NN evaluation (was in autoc-eval.cc)
AircraftState aircraftState;

std::string crashReasonToString(CrashReason type) {
  switch (type) {
  case CrashReason::None: return "None";
  case CrashReason::Boot: return "Boot";
  case CrashReason::Sim: return "Sim";
  case CrashReason::Eval: return "Eval";
  case CrashReason::TimeLimit: return "TimeLimit";
  case CrashReason::RabbitComplete: return "RabbitComplete";
  default: return "*?*";
  }
}

// Physics trace buffer for collecting detailed FDM state
// Cleared at start of each evaluation, sent back with EvalResults only for elite reeval
std::vector<PhysicsTraceEntry> gCurrentPhysicsTrace;

// Extern globals defined in fdm_larcsim.cpp for trace capture
// We set these so the FDM can access worker identity when capturing traces
extern "C" {
  extern int32_t gTraceWorkerId;
  extern int32_t gTraceWorkerPid;
  extern int32_t gTraceEvalCounter;
  extern int32_t gTracePathIndex;
  extern uint32_t gPhysicsStepCounter;
  extern bool gTraceIsEliteReeval;  // Only collect trace when true
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
  // Only collect trace for elite reeval to save CPU
  if (!gTraceIsEliteReeval) {
    return;
  }
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

// Servo slew rate limits in NN command space [-1,1], enforced at command-apply time.
// Full range = 2.0; per-step limit = rate(%/sec) * 2.0 * 0.1s.
static const float SLEW_PITCH    = 0.40f;  // 200%/sec
static const float SLEW_ROLL     = 0.40f;  // 200%/sec
static const float SLEW_THROTTLE = 0.60f;  // 300%/sec
static float prevPitch    = 0.0f;
static float prevRoll     = 0.0f;
static float prevThrottle = 0.0f;

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
  delete socket_;
  socket_ = nullptr;
}

int T_TX_InterfaceAUTOC::init(SimpleXMLTransfer *config)
{
#if DEBUG_TX_INTERFACE > 0
  printf("int T_TX_InterfaceAUTOC::init(SimpleXMLTransfer* config)\n");
#endif
  // Allow runtime override of NN eval cadence and compute latency (sim time).
  gEvalUpdateIntervalMsec = parseIntervalFromEnv("AUTOC_EVAL_INTERVAL_MSEC", EVAL_UPDATE_INTERVAL_MSEC_DEFAULT);
  gComputeLatencyMsec = parseIntervalFromEnv("AUTOC_COMPUTE_LATENCY_MSEC", COMPUTE_LATENCY_MSEC_DEFAULT);
  T_TX_Interface::init(config);

  socket_ = new TcpSocket();
  socket_->connect("127.0.0.1", cfg->callback_port);
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
  diagBuffer[diagIndex] = simTimeMsec;
  diagIndex = (diagIndex + 1) % DIAG_BUFFER_SIZE;
  if (diagCount < DIAG_BUFFER_SIZE) diagCount++;

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
      int start = (diagCount < DIAG_BUFFER_SIZE) ? 0 : diagIndex;
      unsigned long last = diagBuffer[start % DIAG_BUFFER_SIZE];
      for (int i = 0; i < diagCount; i++) {
        unsigned long t = diagBuffer[(start + i) % DIAG_BUFFER_SIZE];
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
      prevPitch = prevRoll = prevThrottle = 0.0f;
      return;
    }

    // Compute dt before updating lastUpdateTimeMsec (used for rabbit odometer advancement)
    gp_scalar evalDtSec = static_cast<gp_scalar>(simTimeMsec - lastUpdateTimeMsec) / 1000.0f;
    lastUpdateTimeMsec = simTimeMsec;
    cycleCounter = 0;

    // reload from the NN code?
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

      evalDataEmpty = false;
      priorPathSelector = -1;
      pathSelector = 0;
      gPendingCommand = PendingCommand{};
      prevPitch = prevRoll = prevThrottle = 0.0f;
      evalResults.aircraftStateList.clear();
      evalResults.crashReasonList.clear();

      // Paths stay at canonical origin (Z=0). Aircraft operates in raw FDM coords.
      // Origin offset (captured at FDM reset) bridges raw→virtual for NN sensor math.
      evalResults.pathList = evalData.pathList;
      path = evalResults.pathList.at(pathSelector);
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
      evalResults.physicsTrace.clear();
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

      // Deserialize NN genome
      if (!nn_deserialize(reinterpret_cast<const uint8_t*>(evalData.gp.data()),
                          evalData.gp.size(), nnGenome)) {
        std::cerr << "Error deserializing NN genome" << std::endl;
        return;
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

      // VARIATIONS1: Set entry and wind variation offsets from scenario
      // These are read by initialize_flight_model() and windfield during reset
      Global::entryHeadingOffset = activeScenario.entryHeadingOffset;
      Global::entryRollOffset = activeScenario.entryRollOffset;
      Global::entryPitchOffset = activeScenario.entryPitchOffset;
      Global::entrySpeedFactor = activeScenario.entrySpeedFactor;
      Global::windDirectionOffset = activeScenario.windDirectionOffset;

      // Entry position offsets (see specs/005-entry-fitness-ramp)
      // Scenario offsets are NED meters; CRRCSim operates in feet
      Global::entryNorthOffset = activeScenario.entryNorthOffset / FEET_TO_METERS;
      Global::entryEastOffset = activeScenario.entryEastOffset / FEET_TO_METERS;
      Global::entryAltOffset = activeScenario.entryAltOffset / FEET_TO_METERS;

#ifdef DETAILED_LOGGING
      std::cerr << "VARIATIONS1: heading=" << (Global::entryHeadingOffset * 180.0/M_PI)
                << "° roll=" << (Global::entryRollOffset * 180.0/M_PI)
                << "° pitch=" << (Global::entryPitchOffset * 180.0/M_PI)
                << "° speed=" << Global::entrySpeedFactor
                << "x wind=" << (Global::windDirectionOffset * 180.0/M_PI) << "°"
                << " posN=" << Global::entryNorthOffset
                << "m posE=" << Global::entryEastOffset
                << "m alt=" << Global::entryAltOffset << "m" << std::endl;
#endif

      // Reset simulation with wind seed
      Global::Simulation->reset(activeScenario.windSeed);
      simCrashed = false;
      lastUpdateTimeMsec = 0;
      pathIndex = 0;
      rabbitOdometer = 0.0f;
      // Generate rabbit speed profile from per-scenario seed + global config
      rabbitSpeedConfig = evalData.rabbitSpeedConfig;
      {
        unsigned int seed = activeScenario.rabbitSpeedSeed;
        double totalDurationSec = SIM_TOTAL_TIME_MSEC / 1000.0;
        rabbitSpeedProfile = generateSpeedProfile(seed, rabbitSpeedConfig, totalDurationSec);
        crrcsimRabbitSpeed = static_cast<gp_scalar>(
            getSpeedAtTime(rabbitSpeedProfile, 0.0));
      }
      gPendingCommand = PendingCommand{};
      prevPitch = prevRoll = prevThrottle = 0.0f;
      aircraftStates.clear();
      aircraftState.clearHistory();  // Reset temporal history for new path
      // Reset commands for new path (NN needs zero-start per path, not per tick)
      aircraftState.setPitchCommand(0.0f);
      aircraftState.setRollCommand(0.0f);
      aircraftState.setThrottleCommand(0.0f);
      gAcroIntegralRoll = gAcroIntegralPitch = gAcroIntegralYaw = 0.0;
      gAcroLastTimeMsec = 0;
      gCurrentPhysicsTrace.clear();  // Clear physics trace for new path

      // Set worker identity globals for FDM trace capture
      gTraceWorkerId = workerIndex;
      gTraceWorkerPid = workerPid;
      gTraceEvalCounter = evalCounter;
      gTracePathIndex = pathSelector;
      gPhysicsStepCounter = 0;  // Reset step counter for new path
      gTraceIsEliteReeval = evalData.isEliteReeval;  // Only collect trace for elite reeval

#ifdef DETAILED_LOGGING
      // Start ordered event logging for this path (noisy RNG trace)
      RandGaussTrace::startEventLog(1000);
      std::cerr << "AUTOC deterministic path start pathSelector=" << pathSelector
                << " pathVariant=" << activeScenario.pathVariantIndex
                << " windVariant=" << activeScenario.windVariantIndex
                << " windSeed=" << activeScenario.windSeed << std::endl;
#endif

      // Record initial aircraft state at time 0 to match path start
      gp_vec3 initialPos{static_cast<gp_scalar>(Global::aircraft->getPos()(0) * FEET_TO_METERS),
                                static_cast<gp_scalar>(Global::aircraft->getPos()(1) * FEET_TO_METERS),
                                static_cast<gp_scalar>(Global::aircraft->getPos()(2) * FEET_TO_METERS)};

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
          static_cast<gp_scalar>(fdm_velocity(0) * FEET_TO_METERS),
          static_cast<gp_scalar>(fdm_velocity(1) * FEET_TO_METERS),
          static_cast<gp_scalar>(fdm_velocity(2) * FEET_TO_METERS)
      };
      gp_scalar initialSpeed = initialVel.norm();

      AircraftState initialState{0, initialSpeed, initialVel, initialQuat, initialPos,
                                 static_cast<gp_scalar>(0.0f), static_cast<gp_scalar>(0.0f), static_cast<gp_scalar>(0.0f), 0};
      initialState.setOriginOffset(initialPos);  // raw→virtual: like xiao arm point
      initialState.setRabbitPosition(path[0].start);
      initialState.setRabbitSpeed(crrcsimRabbitSpeed);
      aircraftStates.push_back(initialState);

      // Carry origin offset to the running aircraftState (used for subsequent ticks)
      aircraftState.setOriginOffset(initialPos);
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
              static_cast<gp_scalar>(vLocal(0) * FEET_TO_METERS),
              static_cast<gp_scalar>(vLocal(1) * FEET_TO_METERS),
              static_cast<gp_scalar>(vLocal(2) * FEET_TO_METERS)};
          CRRCMath::Vector3 vLocalDot = eom01->getVLocalDot();
          sample.vLocalDot = gp_vec3{
              static_cast<gp_scalar>(vLocalDot(0) * FEET_TO_METERS),
              static_cast<gp_scalar>(vLocalDot(1) * FEET_TO_METERS),
              static_cast<gp_scalar>(vLocalDot(2) * FEET_TO_METERS)};
          CRRCMath::Vector3 omegaBody = eom01->getOmegaBody();
          sample.omegaBody = gp_vec3{
              static_cast<gp_scalar>(omegaBody(0)),
              static_cast<gp_scalar>(omegaBody(1)),
              static_cast<gp_scalar>(omegaBody(2))};
          CRRCMath::Vector3 omegaDotBody = eom01->getOmegaDot();
          sample.omegaDotBody = gp_vec3{
              static_cast<gp_scalar>(omegaDotBody(0)),
              static_cast<gp_scalar>(omegaDotBody(1)),
              static_cast<gp_scalar>(omegaDotBody(2))};
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

      // reset commands to default
      inputs->pitch = pitchCommand = 0;
      inputs->aileron = rollCommand = 0;
      inputs->throttle = throttleCommand = 0;
      return;
    }

    // get actual velocity vector from FDM (in feet/s, convert to m/s) - ground speed
    CRRCMath::Vector3 fdm_velocity = Global::aircraft->getFDM()->getVel();
    gp_vec3 velocity_vector{
        fdm_velocity(0) * FEET_TO_METERS,  // North
        fdm_velocity(1) * FEET_TO_METERS,  // East
        fdm_velocity(2) * FEET_TO_METERS   // Down
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
    } else {
      // ERROR: We must use native quaternion for consistency - no fallback allowed
      std::cerr << "FATAL ERROR: FDM is not EOM01 type - cannot access native quaternion!" << std::endl;
      exit(1);
    }
    q.normalize();

    // position — raw FDM coordinates (sim altitude ~-25m NED)
    gp_vec3 p{static_cast<gp_scalar>(Global::aircraft->getPos()(0) * FEET_TO_METERS),
              static_cast<gp_scalar>(Global::aircraft->getPos()(1) * FEET_TO_METERS),
              static_cast<gp_scalar>(Global::aircraft->getPos()(2) * FEET_TO_METERS)};
    if (isnan(p[0]) || isnan(p[1]) || isnan(p[2]))
    {
      p = gp_vec3::Zero();
    }

    // Advance rabbit odometer each eval tick (using dt computed before lastUpdateTimeMsec was updated)
    // Rabbit speed varies over time (from per-scenario profile)
    if (evalDtSec > 0.0f && evalDtSec < 1.0f) {  // Guard against huge jumps
      double simTimeSec = simTimeMsec / 1000.0;
      crrcsimRabbitSpeed = static_cast<gp_scalar>(
          getSpeedAtTime(rabbitSpeedProfile, simTimeSec));
      rabbitOdometer += crrcsimRabbitSpeed * evalDtSec;
    }

    // search for path segment by distance (odometer-based)
    while (pathIndex < static_cast<int>(path.size()) - 2 && path.at(pathIndex).distanceFromStart < rabbitOdometer)
    {
      pathIndex++;
    }

    // Update sim state in AircraftState in-place to preserve temporal history
    aircraftState.setThisPathIndex(pathIndex);
    aircraftState.setRelVel(v);
    aircraftState.setVelocity(velocity_vector);
    aircraftState.setOrientation(q);
    aircraftState.setPosition(p);
    // NN controller reads previous commands as feedback inputs — preserve them.
    aircraftState.setSimTimeMsec(simTimeMsec);
    aircraftState.setRabbitOdometer(rabbitOdometer);
    aircraftState.setRabbitSpeed(crrcsimRabbitSpeed);

    // Body angular rates from FDM (rad/s, standard aerospace RHR)
    if (eom01) {
      CRRCMath::Vector3 omega = eom01->getOmegaBody();
      aircraftState.setGyroRates(gp_vec3{
          static_cast<gp_scalar>(omega(0)),   // p (roll rate)
          static_cast<gp_scalar>(omega(1)),   // q (pitch rate)
          static_cast<gp_scalar>(omega(2))}); // r (yaw rate)
    }

    CrashReason crashReason = CrashReason::None;

    // out of bounds? (raw FDM position)
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
      crashReason = CrashReason::TimeLimit;
    }

    if (pathIndex >= path.size() - 3)
    {
      crashReason = CrashReason::RabbitComplete;
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
      // Only send physics trace data for elite reeval (expensive, only needed for divergence analysis)
      if (evalData.isEliteReeval) {
        if (!debugSamplesCurrentPath.empty()) {
          evalResults.debugSamples.push_back(debugSamplesCurrentPath);
        } else {
          evalResults.debugSamples.emplace_back();
        }
        if (!gCurrentPhysicsTrace.empty()) {
          evalResults.physicsTrace.push_back(gCurrentPhysicsTrace);
        } else {
          evalResults.physicsTrace.emplace_back();
        }
      }
      // Always clear trace buffers (whether sent or not)
      debugSamplesCurrentPath.clear();
      gCurrentPhysicsTrace.clear();

#ifdef DETAILED_LOGGING
      // Dump ordered RandGauss event log for this path (noisy)
      RandGaussTrace::dumpAndClearEventLog();
#endif

      // prepare for the next path if any
      if (++pathSelector < evalResults.pathList.size())
      {
        path = evalResults.pathList.at(pathSelector);
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

    // Capture temporal history before NN evaluation (for GETDPHI_PREV, GETDTHETA_PREV, GETDIST_PREV, etc.)
    {
      VectorPathProvider pathProvider(path, aircraftState.getThisPathIndex());
      gp_scalar dPhi = executeGetDPhi(pathProvider, aircraftState, rabbitOdometer, 0.0f);
      gp_scalar dTheta = executeGetDTheta(pathProvider, aircraftState, rabbitOdometer, 0.0f);
      gp_vec3 targetPos = getInterpolatedTargetPosition(
          pathProvider, rabbitOdometer, 0.0f);
      gp_scalar distance = (targetPos - aircraftState.getVirtualPosition()).norm();
      aircraftState.setRabbitPosition(targetPos);
      aircraftState.recordErrorHistory(dPhi, dTheta, distance, simTimeMsec);
    }

    // Evaluate NN controller
    {
      VectorPathProvider pathProvider(path, aircraftState.getThisPathIndex());
      NNControllerBackend nnBackend(nnGenome);
      nnBackend.evaluate(aircraftState, pathProvider);
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
            static_cast<gp_scalar>(vGround(0) * FEET_TO_METERS),
            static_cast<gp_scalar>(vGround(1) * FEET_TO_METERS),
            static_cast<gp_scalar>(vGround(2) * FEET_TO_METERS)};
        CRRCMath::Vector3 vAir = eom01->getVelRelAirmassVec();
        sample.velRelAirmass = gp_vec3{
            static_cast<gp_scalar>(vAir(0) * FEET_TO_METERS),
            static_cast<gp_scalar>(vAir(1) * FEET_TO_METERS),
            static_cast<gp_scalar>(vAir(2) * FEET_TO_METERS)};
      } else {
        sample.velRelGround = velocity_vector;
        sample.velRelAirmass = gp_vec3::Zero();
      }
      sample.position = p;
      sample.velocity = velocity_vector;
      if (eom01) {
        CRRCMath::Vector3 accelFdm = eom01->getAccel();
        sample.acceleration = gp_vec3{
            static_cast<gp_scalar>(accelFdm(0) * FEET_TO_METERS),
            static_cast<gp_scalar>(accelFdm(1) * FEET_TO_METERS),
            static_cast<gp_scalar>(accelFdm(2) * FEET_TO_METERS)};
        CRRCMath::Vector3 accelPast = eom01->getAccelPast();
        sample.accelPast = gp_vec3{
            static_cast<gp_scalar>(accelPast(0) * FEET_TO_METERS),
            static_cast<gp_scalar>(accelPast(1) * FEET_TO_METERS),
            static_cast<gp_scalar>(accelPast(2) * FEET_TO_METERS)};
        CRRCMath::Vector3 pqr = eom01->getPQR();
        sample.angularRates = gp_vec3{
            static_cast<gp_scalar>(pqr(0)),
            static_cast<gp_scalar>(pqr(1)),
            static_cast<gp_scalar>(pqr(2))};
        CRRCMath::Vector3 omegaDotPast = eom01->getOmegaDotPast();
        sample.angularAccelPast = gp_vec3{
            static_cast<gp_scalar>(omegaDotPast(0)),
            static_cast<gp_scalar>(omegaDotPast(1)),
            static_cast<gp_scalar>(omegaDotPast(2))};
        double quatDotArr[4]; eom01->getQuatDotPast(quatDotArr);
        sample.quatDotPast = gp_quat(
            static_cast<gp_scalar>(quatDotArr[0]),
            static_cast<gp_scalar>(quatDotArr[1]),
            static_cast<gp_scalar>(quatDotArr[2]),
            static_cast<gp_scalar>(quatDotArr[3]));
        CRRCMath::Vector3 windBodyFdm = eom01->getWindBody();
        sample.windBody = gp_vec3{
            static_cast<gp_scalar>(windBodyFdm(0) * FEET_TO_METERS),
            static_cast<gp_scalar>(windBodyFdm(1) * FEET_TO_METERS),
            static_cast<gp_scalar>(windBodyFdm(2) * FEET_TO_METERS)};
        sample.massKg = static_cast<gp_scalar>(eom01->getMass());
        sample.density = static_cast<gp_scalar>(eom01->getDensity());
        sample.gravity = static_cast<gp_scalar>(eom01->getGravity());
        sample.alpha = static_cast<gp_scalar>(eom01->getAlpha());
        sample.beta = static_cast<gp_scalar>(eom01->getBeta());
        sample.vRelWind = static_cast<gp_scalar>(eom01->getVRelWind());
        CRRCMath::Vector3 vLocal = eom01->getVLocal();
        sample.vLocal = gp_vec3{
            static_cast<gp_scalar>(vLocal(0) * FEET_TO_METERS),
            static_cast<gp_scalar>(vLocal(1) * FEET_TO_METERS),
            static_cast<gp_scalar>(vLocal(2) * FEET_TO_METERS)};
        CRRCMath::Vector3 vLocalDot = eom01->getVLocalDot();
        sample.vLocalDot = gp_vec3{
            static_cast<gp_scalar>(vLocalDot(0) * FEET_TO_METERS),
            static_cast<gp_scalar>(vLocalDot(1) * FEET_TO_METERS),
            static_cast<gp_scalar>(vLocalDot(2) * FEET_TO_METERS)};
        CRRCMath::Vector3 omegaBody = eom01->getOmegaBody();
        sample.omegaBody = gp_vec3{
            static_cast<gp_scalar>(omegaBody(0)),
            static_cast<gp_scalar>(omegaBody(1)),
            static_cast<gp_scalar>(omegaBody(2))};
        CRRCMath::Vector3 omegaDotBody = eom01->getOmegaDot();
        sample.omegaDotBody = gp_vec3{
            static_cast<gp_scalar>(omegaDotBody(0)),
            static_cast<gp_scalar>(omegaDotBody(1)),
            static_cast<gp_scalar>(omegaDotBody(2))};
        sample.latGeoc = static_cast<gp_scalar>(eom01->getLatGeocentric());
        sample.lonGeoc = static_cast<gp_scalar>(eom01->getLonGeocentric());
        sample.radiusToVehicle = static_cast<gp_scalar>(eom01->getRadiusToVehicle() * FEET_TO_METERS);
        CRRCMath::Vector3 localAirmass = eom01->getLastLocalAirmass();
        sample.localAirmass = gp_vec3{
            static_cast<gp_scalar>(localAirmass(0) * FEET_TO_METERS),
            static_cast<gp_scalar>(localAirmass(1) * FEET_TO_METERS),
            static_cast<gp_scalar>(localAirmass(2) * FEET_TO_METERS)};
        CRRCMath::Vector3 gustBody = eom01->getLastGustBody();
        sample.gustBody = gp_vec3{
            static_cast<gp_scalar>(gustBody(0) * FEET_TO_METERS),
            static_cast<gp_scalar>(gustBody(1) * FEET_TO_METERS),
            static_cast<gp_scalar>(gustBody(2) * FEET_TO_METERS)};
        CRRCMath::Vector3 forceBody = eom01->getLastForceBody();
        sample.forceBody = gp_vec3{
            static_cast<gp_scalar>(forceBody(0) * FEET_TO_METERS),
            static_cast<gp_scalar>(forceBody(1) * FEET_TO_METERS),
            static_cast<gp_scalar>(forceBody(2) * FEET_TO_METERS)};
        CRRCMath::Vector3 momentBody = eom01->getLastMomentBody();
        sample.momentBody = gp_vec3{
            static_cast<gp_scalar>(momentBody(0)),
            static_cast<gp_scalar>(momentBody(1)),
            static_cast<gp_scalar>(momentBody(2))};
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
    // Log key sensor values that NN can access for diagnostics
    {
      char tbuf[100];
      gp_vec3 velocity_body = aircraftState.getOrientation().inverse() * aircraftState.getVelocity();
      gp_scalar alpha = std::atan2(-velocity_body.z(), velocity_body.x());
      gp_scalar beta = std::atan2(velocity_body.y(), velocity_body.x());
      gp_scalar vel = aircraftState.getRelVel();
      gp_scalar dhome = aircraftState.getVirtualPosition().norm();  // distance from virtual origin

      gp_vec3 euler = aircraftState.getOrientation().toRotationMatrix().eulerAngles(2, 1, 0);
      gp_scalar roll_rad = euler[2];
      gp_scalar pitch_rad = euler[1];

      printf("%s: nn_sensors: vel=%8.2f alpha_deg=%8.2f beta_deg=%8.2f dhome=%8.2f velx=%8.2f vely=%8.2f velz=%8.2f roll_rad=%8.4f pitch_rad=%8.4f\n",
             get_iso8601_timestamp(tbuf, sizeof(tbuf)),
             vel, alpha * 180.0/M_PI, beta * 180.0/M_PI, dhome,
             aircraftState.getVelocity().x(), aircraftState.getVelocity().y(), aircraftState.getVelocity().z(),
             roll_rad, pitch_rad);
    }
#endif
  }

  // Apply pending commands when latency expires, with servo slew rate limiting.
  if (gPendingCommand.valid && simTimeMsec >= gPendingCommand.readyTimeMsec) {
    pitchCommand    = std::clamp(gPendingCommand.pitch,    prevPitch    - SLEW_PITCH,    prevPitch    + SLEW_PITCH);
    rollCommand     = std::clamp(gPendingCommand.roll,     prevRoll     - SLEW_ROLL,     prevRoll     + SLEW_ROLL);
    throttleCommand = std::clamp(gPendingCommand.throttle, prevThrottle - SLEW_THROTTLE, prevThrottle + SLEW_THROTTLE);
    prevPitch    = pitchCommand;
    prevRoll     = rollCommand;
    prevThrottle = throttleCommand;
    gPendingCommand.valid = false;
  }

  // ACRO rate PID: convert NN rate commands to surface deflections.
  // NN output [-1,1] is the desired angular rate as a fraction of max rate.
  // PID compares desired vs actual body rate (both rad/s) → surface deflection [-1,1].
  //
  // All rate math is in rad/s to match:
  //   - FDM body rates (getOmegaBody() returns rad/s)
  //   - NN gyro inputs (AircraftState gyroRates_ in rad/s)
  //   - AircraftState convention (see COORDINATE_CONVENTIONS.md)
  //
  // The PID gains (ACRO_FF/P/I) and ACRO_PID_SCALE are empirical tuning knobs
  // for the CRRCSim FDM, NOT direct copies of INAV's gains. INAV's gains are
  // tuned for its own internal units and servo response. CRRCSim needs its own
  // tuning to produce similar rate-tracking behavior.
  {
    EOM01* eom01_acro = dynamic_cast<EOM01*>(Global::aircraft->getFDM());
    if (eom01_acro) {
      CRRCMath::Vector3 omega = eom01_acro->getOmegaBody();  // rad/s
      double pRadS = omega(0);  // roll rate
      double qRadS = omega(1);  // pitch rate
      double rRadS = omega(2);  // yaw rate

      // NN commands [-1,1] → desired rates (rad/s)
      double maxRollRadS  = ACRO_MAX_RATE_ROLL  * M_PI / 180.0;  // 560 deg/s → ~9.77 rad/s
      double maxPitchRadS = ACRO_MAX_RATE_PITCH * M_PI / 180.0;  // 400 deg/s → ~6.98 rad/s
      double desiredRoll  = rollCommand  * maxRollRadS;
      double desiredPitch = pitchCommand * maxPitchRadS;

      // Rate error (rad/s)
      double errRoll  = desiredRoll  - pRadS;
      double errPitch = desiredPitch - qRadS;

      // Integrate (with anti-windup clamp in rad)
      double dt = (gAcroLastTimeMsec > 0 && simTimeMsec > gAcroLastTimeMsec)
                  ? (simTimeMsec - gAcroLastTimeMsec) / 1000.0
                  : 0.01;  // fallback 10ms
      gAcroLastTimeMsec = simTimeMsec;
      gAcroIntegralRoll  = std::clamp(gAcroIntegralRoll  + errRoll  * dt, -10.0, 10.0);
      gAcroIntegralPitch = std::clamp(gAcroIntegralPitch + errPitch * dt, -10.0, 10.0);

      // PID output: FF*command + P*error + I*integral (no D term)
      // Output normalized to ~[-1,1] by ACRO_PID_SCALE
      double outRoll  = (ACRO_FF_ROLL  * desiredRoll  + ACRO_P_ROLL  * errRoll  + ACRO_I_ROLL  * gAcroIntegralRoll)  / ACRO_PID_SCALE;
      double outPitch = (ACRO_FF_PITCH * desiredPitch + ACRO_P_PITCH * errPitch + ACRO_I_PITCH * gAcroIntegralPitch) / ACRO_PID_SCALE;

      // Replace NN commands with PID output (clamped to [-1,1])
      pitchCommand    = std::clamp(outPitch, -1.0, 1.0);
      rollCommand     = std::clamp(outRoll,  -1.0, 1.0);
      // throttle passes through directly (no rate PID for throttle)
    }
  }

  // convert cached values to crrcsim scales and return every frame
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
