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

static bool gInDeterministicTest = false;

// Global aircraftState used by NN evaluation (was in autoc-eval.cc)
AircraftState aircraftState;

// Raw→virtual origin offset for current path. Captured at path start from FDM position.
// Used to convert raw FDM position to virtual coordinates for AircraftState.
gp_vec3 pathOriginOffset = gp_vec3::Zero();

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

  // Engage delay: coast time before NN outputs reach surfaces (models INAV handoff)
  {
    const char* env = std::getenv("AUTOC_ENGAGE_DELAY_MSEC");
    if (env && *env != '\0') {
      engageDelayMsec = strtoul(env, nullptr, 10);
    }
    std::cerr << "[AUTOC] EngageDelayMsec=" << engageDelayMsec << std::endl;
  }

  // Compute frame-counter cadence triple and enforce integrality at startup.
  // Constraint: outer cycleLength_ms must divide gEvalUpdateIntervalMsec
  // exactly. CTime already rounds cycleLength to a multiple of
  // (Global::dt * 1000). See spec 024 WI4.
  {
    isHeadless = (config->getInt("video.enabled", 1) == 0);
    SimpleXMLTransfer* video = config->getChild("video", true);
    const int fps = video->attributeAsInt("fps", 0);
    const double dtMs = 1000.0 * static_cast<double>(Global::dt);
    const unsigned long cycleLengthMs =
      (fps > 0 && dtMs > 0.0)
        ? static_cast<unsigned long>(static_cast<int>(1000.0 / fps / dtMs) * dtMs + 0.5)
        : 0UL;
    if (cycleLengthMs == 0 || gEvalUpdateIntervalMsec % cycleLengthMs != 0) {
      std::cerr << "[AUTOC] FATAL: cadence triple not integral. "
                << "video.fps=" << fps
                << " Global::dt=" << Global::dt
                << " cycleLengthMs=" << cycleLengthMs
                << " gEvalUpdateIntervalMsec=" << gEvalUpdateIntervalMsec
                << " — evalInterval must be an integer multiple of cycleLength."
                << std::endl;
      std::exit(1);
    }
    framesPerEval = gEvalUpdateIntervalMsec / cycleLengthMs;
    std::cerr << "[AUTOC] cadence: video.fps=" << fps
              << " dt=" << Global::dt
              << " cycleLengthMs=" << cycleLengthMs
              << " evalIntervalMsec=" << gEvalUpdateIntervalMsec
              << " framesPerEval=" << framesPerEval
              << " headless=" << (isHeadless ? "true" : "false")
              << std::endl;
  }

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

  // Frame-counter cadence (both headless and video):
  //  - Outer frame is a fixed cycleLength_ms (CTime). framesPerEval was
  //    validated at init to divide gEvalUpdateIntervalMsec exactly, so every
  //    framesPerEval-th outer frame fires eval with zero drift.
  //  - simTimeMsec is stamped into diagBuffer for drift logging in video mode.
  //  - A "way late" stall (wall-clock pause in video mode, computer hiccup,
  //    etc.) shows up as a simTime leap > SIM_MAX_INTERVAL_MSEC and is caught
  //    by the jump reset below, which clears counters and bookkeeping.
  bool shouldEval = (++cycleCounter >= framesPerEval);
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
      evalResults.aircraftStateList.clear();
      evalResults.crashReasonList.clear();

      // Paths stay at canonical origin (Z=0). Aircraft position stored as virtual
      // (raw - pathOriginOffset). Origin offset captured at FDM reset per path.
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

      // Build/rebuild the NN controller for the new genome. Recurrent
      // hidden state (spec 027) lives inside the backend and is reset
      // per-span below.
      nnController_ = std::make_unique<NNControllerBackend>(nnGenome);

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
      rabbitSpeedConfig = evalData.rabbitSpeedConfig;
      gPendingCommand = PendingCommand{};
      aircraftStates.clear();
      // 030 M11.preA — clear per-path tracker buffers (no-op in pathgen mode)
      trackerCameraViewSteps_.clear();
      trackerTargetSampleSteps_.clear();
      // 030 M11.preA — Mode-aware scenario bookkeeping. Pathgen needs the
      // rabbit-speed profile, engage-delay timing, and cruise throttle for
      // INAV-handoff fidelity. Tracker mode uses none of these (no synthetic
      // rabbit, NN commands fire from tick 0). 2026-05-08 fix: previously
      // generateSpeedProfile ran unconditionally and the result was
      // discarded in tracker mode — wasted CPU + memory. Moved into else.
      const bool trackerModeActive = (evalData.mode == Mode::TRACKER);
      if (trackerModeActive) {
        engageDelayTicksRemaining = 0;  // Chase commands fire from tick 0
        engageCoastThrottle = static_cast<gp_scalar>(0.0);
        rabbitSpeedProfile.clear();
        crrcsimRabbitSpeed = 0.0f;
      } else {
        // Pathgen mode (existing): generate per-scenario rabbit-speed profile
        // from autoc-side seed; engage delay + cruise throttle for INAV
        // handoff fidelity.
        {
          unsigned int seed = activeScenario.rabbitSpeedSeed;
          double totalDurationSec = SIM_TOTAL_TIME_MSEC / 1000.0;
          rabbitSpeedProfile = generateSpeedProfile(seed, rabbitSpeedConfig, totalDurationSec);
          crrcsimRabbitSpeed = static_cast<gp_scalar>(
              getSpeedAtTime(rabbitSpeedProfile, 0.0));
        }
        engageDelayTicksRemaining = static_cast<int>(
            (engageDelayMsec + gEvalUpdateIntervalMsec - 1) / gEvalUpdateIntervalMsec);
        engageCoastThrottle = static_cast<gp_scalar>(
            CLAMP_DEF(2.0 * (activeScenario.entrySpeedFactor - 1.0), -1.0, 1.0));
      }
      // Zero recurrent NN state at span start (spec 027 Q4: h_t resets
      // on new scenario; no-op for feedforward genomes).
      if (nnController_) nnController_->reset();
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

      // Canonical raw→virtual origin offset (NOT the actual FDM start position).
      // Entry variations intentionally start the craft off-target — that offset
      // must be visible to the fitness function as the aircraft's deviation from
      // the path origin, not absorbed into pathOriginOffset.
      pathOriginOffset = gp_vec3(0.0f, 0.0f, SIM_INITIAL_ALTITUDE);

      // Virtual initial position = entry-variation deviation from canonical start.
      // Without variations: (0,0,0). With variations: (entryNorth, entryEast, entryAlt).
      gp_vec3 virtualInitialPos = initialPos - pathOriginOffset;
      AircraftState initialState{0, initialSpeed, initialVel, initialQuat, virtualInitialPos,
                                 static_cast<gp_scalar>(0.0f), static_cast<gp_scalar>(0.0f), static_cast<gp_scalar>(0.0f), 0};
      initialState.setRabbitPosition(path[0].start);
      initialState.setRabbitSpeed(crrcsimRabbitSpeed);

      // Copy initial state to the global aircraftState BEFORE history pre-fill
      // so resetHistory() uses the correct position/orientation (not leftover
      // from the previous scenario).
      aircraftState = initialState;

      // Pre-fill history buffer with initial geometry so the NN starts with
      // consistent direction cosines instead of zeros (pathgen-mode only;
      // tracker mode uses TrackerHistoryWindow inside trackerHelper_ instead).
      {
        gp_vec3 tangent;
        if (path.size() > 1)
          tangent = path[1].start - path[0].start;
        else
          tangent = gp_vec3::UnitX();
        double tn = tangent.norm();
        if (tn > 1e-6) tangent = tangent / tn;
        else tangent = gp_vec3::UnitX();
        aircraftState.resetHistory(path[0].start, tangent);
      }

      // 030 M11.preA — Tracker-mode helper scenario init (after chase
      // pose is built from FDM post-reset state). Pre-fills 6-slot beacon
      // history with source[0] projection × 6, seeds crash hull PRNG,
      // resets NN recurrent state. No-op when mode != "tracker".
      if (trackerModeActive
          && pathSelector < static_cast<int>(evalData.sourceList.size())
          && nnController_) {
        const auto& source = evalData.sourceList.at(pathSelector);
        if (!source.samples.empty()) {
          trackerHelper_.initScenario(source, activeScenario, evalData,
                                      aircraftState, *nnController_);
        }
      }

      aircraftStates.push_back(aircraftState);
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
    // 030 M11.preA — Rabbit odometer + path index advancement is pathgen-
    // specific. In tracker mode the source-cursor inside trackerHelper_
    // drives target progression instead.
    const bool trackerActiveTick = (evalData.mode == Mode::TRACKER);
    if (!trackerActiveTick) {
      // Pathgen mode (existing): advance rabbit odometer + walk path.
      if (evalDtSec > 0.0f && evalDtSec < 1.0f) {  // Guard against huge jumps
        double simTimeSec = simTimeMsec / 1000.0;
        crrcsimRabbitSpeed = static_cast<gp_scalar>(
            getSpeedAtTime(rabbitSpeedProfile, simTimeSec));
        rabbitOdometer += crrcsimRabbitSpeed * evalDtSec;
      }
      while (pathIndex < static_cast<int>(path.size()) - 2 &&
             path.at(pathIndex).distanceFromStart < rabbitOdometer) {
        pathIndex++;
      }
    }

    // Update sim state in AircraftState in-place to preserve temporal history
    aircraftState.setThisPathIndex(pathIndex);
    aircraftState.setRelVel(v);
    aircraftState.setVelocity(velocity_vector);
    aircraftState.setOrientation(q);
    aircraftState.setPosition(p - pathOriginOffset);  // store virtual position
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

    // 030 M11.preA — Pathgen uses legacy SIM_PATH_RADIUS_LIMIT bounds +
    // RabbitComplete (chase reached path end). Tracker uses FlightArena
    // bounds via helper.tick() (already includes arena-egress + hull-strike
    // checks), and source-exhaustion via helper.sourceExhausted().
    if (!trackerActiveTick) {
      // out of bounds? Use raw FDM position (not virtual from aircraftState)
      gp_scalar distanceFromOrigin = std::sqrt(p[0] * p[0] + p[1] * p[1]);
      if (p[2] < SIM_MAX_ELEVATION || // too high
          p[2] > SIM_MIN_ELEVATION || // too low
          distanceFromOrigin > SIM_PATH_RADIUS_LIMIT) {  // too far
        crashReason = CrashReason::Eval;
      }
    }

    // sim crashed? — applies in BOTH modes (FDM-physics-NaN propagation, etc.)
    if (Global::Simulation->getState() == STATE_CRASHED)
    {
      crashReason = CrashReason::Sim;
    }

    // Time limit applies in both modes.
    if (simTimeMsec > SIM_TOTAL_TIME_MSEC)
    {
      crashReason = CrashReason::TimeLimit;
    }

    if (!trackerActiveTick) {
      // Pathgen RabbitComplete (chase walked path to end).
      if (pathIndex >= path.size() - 3) {
        crashReason = CrashReason::RabbitComplete;
      }
    } else {
      // Tracker source-exhaustion = "out of source samples" — semantically
      // equivalent to RabbitComplete (scenario ran to end without crash).
      if (trackerHelper_.sourceExhausted()) {
        crashReason = CrashReason::RabbitComplete;
      }
    }

    // crashed or out of time or off the end of the list
    if (crashReason != CrashReason::None)
    {
#ifdef DETAILED_LOGGING
      std::cout << "sim: " << crashReasonToString(crashReason) << " time: " << simTimeMsec << " idx: " << pathIndex << " size: " << path.size() << std::endl;
#endif

      // save the crash state
      evalResults.crashReasonList.push_back(crashReason);

      // save the results list (with origin offset for renderer raw reconstruction)
      ScenarioMetadata pathMeta = scenarioForPathIndex(evalData, static_cast<size_t>(pathSelector));
      pathMeta.originOffset = pathOriginOffset;
      evalResults.scenarioList.push_back(pathMeta);

      std::vector<AircraftState> aircraftStatesCopy = aircraftStates;
      evalResults.aircraftStateList.push_back(aircraftStatesCopy);
      aircraftStates.clear();

      // 030 M11.preA — Per-scenario tracker buffers into v=2 dmp fields.
      // Pathgen mode pushes empty vectors (tracker buffers stay clear),
      // matching minisim's behavior where pathgen dmps load with empty
      // cameraViewList/targetTrajectoryList per the M8a schema contract.
      evalResults.cameraViewList.push_back(trackerCameraViewSteps_);
      evalResults.targetTrajectoryList.push_back(trackerTargetSampleSteps_);
      trackerCameraViewSteps_.clear();
      trackerTargetSampleSteps_.clear();
      // Per-scenario telemetry counters (M7d.b). hullStrikeCount = hull
      // p_crash fires this scenario; arenaEgressCount = 1 if scenario
      // terminated via Eval (arena egress) else 0.
      evalResults.hullStrikeCount.push_back(
          (evalData.mode == Mode::TRACKER) ? trackerHelper_.hullFiredCount() : 0);
      evalResults.arenaEgressCount.push_back(
          (crashReason == CrashReason::Eval) ? 1 : 0);
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

    // 030 M11.preA — Mode-aware NN evaluation block.
    //   Tracker mode: helper.tick() projects beacons + shifts history,
    //     gathers TrackerInputs (45 floats including arena-distance), runs
    //     evaluateTracker, returns per-tick crash signal (HullStrike or
    //     arena egress).
    //   Pathgen mode (existing): direction-cosine target-history capture,
    //     nnController_->evaluate against rabbit-odometer-driven path.
    if (evalData.mode == Mode::TRACKER) {
      if (nnController_) {
        CrashReason trackerCrash =
            trackerHelper_.tick(aircraftState, *nnController_, evalData);
        if (trackerCrash != CrashReason::None && crashReason == CrashReason::None) {
          crashReason = trackerCrash;
        }
      }
      // Per-tick M2 dmp recording — push helper's lastCameraView /
      // lastTargetSample into per-path buffers in lockstep with the
      // aircraftStates push below. Pushed into evalResults at path-end.
      trackerCameraViewSteps_.push_back(trackerHelper_.lastCameraView());
      trackerTargetSampleSteps_.push_back(trackerHelper_.lastTargetSample());
    } else {
      // Pathgen mode (unchanged): capture temporal history (direction
      // cosines, 023) before NN evaluation.
      {
        VectorPathProvider pathProvider(path, aircraftState.getThisPathIndex());
        gp_vec3 targetPos = getInterpolatedTargetPosition(
            pathProvider, rabbitOdometer, 0.0f);
        gp_vec3 craftToTarget = targetPos - aircraftState.getPosition();
        gp_vec3 target_local = aircraftState.getOrientation().inverse() * craftToTarget;
        float distance = static_cast<float>(target_local.norm());

        // Path tangent for singularity fallback
        gp_vec3 posAhead = getInterpolatedTargetPosition(pathProvider, rabbitOdometer, 0.5f);
        gp_vec3 tangent = posAhead - targetPos;
        double tn = tangent.norm();
        gp_vec3 tangent_body = (tn > 1e-6)
            ? aircraftState.getOrientation().inverse() * (tangent / tn)
            : gp_vec3::UnitX();

        gp_vec3 dir = computeTargetDir(target_local, distance, tangent_body);
        aircraftState.setRabbitPosition(targetPos);
        aircraftState.recordErrorHistory(dir, distance, simTimeMsec);
      }

      // Evaluate pathgen NN controller. Use the per-span member
      // `nnController_` so recurrent hidden state (spec 027) persists
      // across ticks. Fired on NN-eval cadence only — the `shouldEval`
      // gate above enforces this.
      if (nnController_) {
        VectorPathProvider pathProvider(path, aircraftState.getThisPathIndex());
        nnController_->evaluate(aircraftState, pathProvider);
      }
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

    // Stage commands to be applied after compute latency.
    // During engage delay window: hold stick centered (pitch/roll=0) with
    // cruise throttle derived from entry speed variation — aircraft coasts
    // on momentum, matching real handoff.
    if (engageDelayTicksRemaining > 0) {
      gPendingCommand.pitch = 0.0f;
      gPendingCommand.roll = 0.0f;
      gPendingCommand.throttle = engageCoastThrottle;
      engageDelayTicksRemaining--;
    } else {
      gPendingCommand.pitch = aircraftState.getPitchCommand();
      gPendingCommand.roll = aircraftState.getRollCommand();
      gPendingCommand.throttle = aircraftState.getThrottleCommand();
    }
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
      gp_scalar dhome = aircraftState.getPosition().norm();  // distance from virtual origin

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

  // Apply pending commands when latency expires (no slew rate limiting — matches
  // hardware path where Xiao writes NN outputs directly to MSP_SET_RAW_RC).
  if (gPendingCommand.valid && simTimeMsec >= gPendingCommand.readyTimeMsec) {
    pitchCommand    = gPendingCommand.pitch;
    rollCommand     = gPendingCommand.roll;
    throttleCommand = gPendingCommand.throttle;
    gPendingCommand.valid = false;
  }

  // NN outputs are direct surface commands (unity pass-through). 026's
  // external ACRO PID experiment went NO-GO — see
  // specs/026-nn-temporal-state/findings.md. Only thing kept is
  // observational diagnostic capture: rateCmd expresses what the NN
  // output would mean as a body rate if a PID were active, rateAch is
  // the actual body rate, FF term mirrors the unity command. P/I/integ
  // all zero.
  {
    EOM01* eom01_acro = dynamic_cast<EOM01*>(Global::aircraft->getFDM());
    if (eom01_acro && !aircraftStates.empty()) {
      CRRCMath::Vector3 omega = eom01_acro->getOmegaBody();
      const double maxRollRadS  = ACRO_MAX_RATE_ROLL  * M_PI / 180.0;
      const double maxPitchRadS = ACRO_MAX_RATE_PITCH * M_PI / 180.0;
      PidInternals pid;
      pid.rateCmdP = static_cast<float>(rollCommand  * maxRollRadS);
      pid.rateCmdQ = static_cast<float>(pitchCommand * maxPitchRadS);
      pid.rateAchP = static_cast<float>(omega(0));
      pid.rateAchQ = static_cast<float>(omega(1));
      pid.ffP = static_cast<float>(rollCommand);
      pid.ffQ = static_cast<float>(pitchCommand);
      pid.sat = (std::abs(pitchCommand) >= 1.0 ? 0x1 : 0)
              | (std::abs(rollCommand)  >= 1.0 ? 0x2 : 0);
      aircraftStates.back().setPidInternals(pid);
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
