// 030 M11.preA — CRRCSim tracker-mode helper class implementation.
//
// Body intentionally mirrors minisim's TrackerStepper (src/eval/tracker_stepper.cc)
// at the per-tick logic level. Adapted for crrcsim:
//   - No state_ owned by helper — chase state lives in inputdev_autoc.cpp's
//     `aircraftState` global, populated from FDM each NN tick. Helper takes
//     it by reference each call.
//   - No physics advance in tick() — crrcsim FDM advances on its own schedule;
//     helper just reads chaseState (post-FDM-step) and writes NN commands
//     back to it.
//   - Cursor starts at 0 (no pre-roll) per M11.preA "match M1 init" decision.

#include "crrcsim_tracker_helper.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "autoc/eval/arena.h"
#include "autoc/eval/tracker_tick_rule.h"  // 040 T011 — single-sourced per-tick rule
#include "autoc/eval/trail_rabbit.h"
#include "autoc/util/scenario_prng.h"     // 033 — deriveClassSubSeeds

using autoc::eval::CrashHull;
using autoc::eval::ArenaEgressKind;
using autoc::eval::checkArenaBounds;
using autoc::eval::arenaEgressToCrashReason;
using autoc::eval::projectBeacon;
using autoc::eval::computeTrailRabbit;
using autoc::eval::isInsideHull;
using autoc::eval::didCrashFire;
using autoc::eval::ProjectionInput;
using autoc::eval::BeaconObservation;

void CrrcsimTrackerHelper::initScenario(const SourceScenarioTrajectory& source,
                                        const ScenarioMetadata& meta,
                                        const WorkerInit& init,
                                        AircraftState& chaseState,
                                        NNControllerBackend& nn) {
    source_ = &source;
    cursor_ = 0;
    hull_fired_count_ = 0;

    // 030 V1 priming — CrashHull config from WorkerInit (autoc-side reads
    // autoc-tracker.ini and ships once per worker, not per eval).
    crash_hull_ = CrashHull{};
    crash_hull_.sphere_radius_m = init.crashHullRadius;

    // 033 cleanup — crash-hull PRNG seed now derives from the RABBIT class
    // sub-PRNG (per contracts/scenario_prng_chain.md — M2 crash-hull is
    // a rabbit-class consumer). Pre-033 used meta.windSeed; that field
    // has been removed. deriveClassSubSeeds(scenarioSeed) is the canonical
    // route — autoc-side minisim TrackerStepper uses the same derivation
    // path (deterministic across processes for the same scenarioSeed).
    // anti-zero guard preserved (Park-Miller LCG breaks at state=0).
    const auto subseeds = autoc::util::deriveClassSubSeeds(meta.scenarioSeed);
    const uint32_t seed = subseeds.rabbit;
    prng_state_ = ((seed == 0 ? 0xC0FFEEu : seed % 0x7FFFFFFFu)) | 1u;

    // Reset NN recurrent state at scenario start (no-op for feedforward).
    nn.reset();

    // 038 P0-D FR-P0H (A) — reset situational-awareness state per scenario
    // (FR-030 determinism). Advanced only on real ticks in tick(), NOT during
    // the history pre-fill below. Mirrors src/eval/tracker_stepper.cc.
    // 040 T011 (FR-020a) — reset ALL carried perception state through one
    // call so a future addition (the acquisition state machine) is picked up
    // by both execution paths without either being edited.

    // 037 T022 — fail loud on a source library whose tick spacing does not
    // match the compiled cadence (the caller advances one SIM_TIME_STEP_MSEC
    // of chase physics per source tick; a 100 ms-recorded library at a 50 ms
    // cadence would silently play the target at 2× speed). Mirrors
    // src/eval/tracker_stepper.cc::initScenario.
    // 038 P0-D-1: STRICT single-gap check restored. simTimeMsec is now
    // round()-stamped (SimStateHandler::getSimulationTimeSinceReset) → exact
    // 50 ms gaps, so the first gap is a faithful cadence probe again. (The
    // 2026-06-15 average-gap workaround tolerated the old truncation jitter of
    // 49/50/51 ms; that jitter is fixed at the stamp now.)
    if (source_->samples.size() >= 2) {
        const auto& s = source_->samples;
        const long firstGapMsec =
            std::lround(s[1].simTimeMsec - s[0].simTimeMsec);
        if (firstGapMsec != SIM_TIME_STEP_MSEC) {
            throw std::runtime_error(
                "CrrcsimTrackerHelper: source trajectory tick spacing " +
                std::to_string(firstGapMsec) + " ms != compiled SIM_TIME_STEP_MSEC " +
                std::to_string(SIM_TIME_STEP_MSEC) +
                " ms — rebake the M2 source library at the current cadence.");
        }
    }

    // Pre-fill history with source[0] projection replicated across the WHOLE
    // observation ring (037: depth grew with the R5 lag window), so the NN
    // sees a coherent stationary-source history at first tick. Mirrors the
    // minisim TrackerStepper init for the pre_roll == 0 case.
    // 040 US6 — capture this scenario's camera draw BEFORE the history pre-fill,
    // so the pre-filled ticks see the same camera the scenario will actually fly.
    // Filling with the nominal camera and then switching would hand the NN a
    // discontinuity at tick 1 that no real airframe has.
    //
    // Indexed out of WorkerInit (primed once), NOT read from ScenarioMetadata:
    // that struct is persisted in every dmp, and putting the draws there orphaned
    // the pinned M1 source. Out-of-range ⇒ the nominal camera, which is also the
    // pathgen / camera-variation-off path.
    {
        const size_t idx = static_cast<size_t>(source.sourceScenarioIndex);
        camera_variation_ = (idx < init.cameraVariations.size())
                                ? init.cameraVariations[idx]
                                : autoc::eval::CameraDeltas{};
    }

    autoc::eval::resetPerceptionState(obs_ring_, sa_state_, perception_carry_);
    if (!source_->samples.empty()) {
        const SourceTickSample& first = source_->samples.front();
        for (int i = 0; i < TrackerObservationRing::kDepth; ++i) {
            projectAndShiftHistory(first, chaseState, init);
        }
    }
    // Cursor stays at 0 — first tick() consumes source[0] (the same sample
    // we just used to pre-fill history; that's fine, history shifts cleanly).
}

void CrrcsimTrackerHelper::projectAndShiftHistory(const SourceTickSample& target,
                                                  const AircraftState& chaseState,
                                                  const WorkerInit& init) {
    // 037 T022 — observations land in the deep ring; the 6-slot gather view
    // (history_) is materialized at the R5 lag offsets at the end of this
    // function. (Pre-037: 6-slot shift-left register.)

    // 040 T011 (FR-031) — the projection + CEP-gated-separation rule now lives
    // once in src/eval/tracker_tick_rule.cc, consumed identically by this
    // PRODUCTION tick and by the test-only TrackerStepper reference. Config
    // still arrives via WorkerInit (no ConfigManager on the crrcsim worker).
    autoc::eval::TickRuleConfig rule_cfg;
    rule_cfg.camera = init.cameraConfig;
    rule_cfg.beacon_left = init.beaconLeftConfig;
    rule_cfg.beacon_right = init.beaconRightConfig;
    rule_cfg.airframe = init.airframeObstruction;
    rule_cfg.cep_gate_threshold = static_cast<gp_scalar>(init.cepGateThreshold);
    // 040 US4 — link budget + acquisition machine, both from WorkerInit. The
    // worker has no ConfigManager, so these arrive over the RPC like every other
    // scenario-invariant config.
    rule_cfg.signal = init.signalConfig;
    rule_cfg.acquisition = init.acquisitionConfig;
    rule_cfg.control_interval_ms =
        static_cast<gp_scalar>(init.controlIntervalMsec);
    // Nominal until applyCameraVariation moves it (T074: obstruction only).
    rule_cfg.obstruction_mount_offset = init.cameraConfig.mount_offset_body;
    autoc::eval::applyCameraVariation(rule_cfg, camera_variation_);

    const autoc::eval::PerceptionTickResult tick_result =
        autoc::eval::projectPerceptionTick(chaseState, target, rule_cfg,
                                           perception_carry_);
    const BeaconObservation& left = tick_result.left;
    const BeaconObservation& right = tick_result.right;

    obs_ring_.push(tick_result.record);
    obs_ring_.materialize(history_);

    // M2 dmp recording — mirror minisim's M8b populate.
    last_camera_view_.camera_pose_world_pos =
        chaseState.getPosition() + chaseState.getOrientation() *
        init.cameraConfig.mount_offset_body;
    // 040 US6 FIX (2026-08-02) — record the VARIED orientation from rule_cfg,
    // not the nominal one from WorkerInit.
    //
    // This recorded the nominal pose while the BEARINGS were being projected
    // through the varied one, so the dmp claimed the camera pointed down the
    // nominal boresight when it was actually up to 20 deg off. Two visible
    // consequences: the renderer recovers mountQ = chase^-1 * (chase * nominal)
    // = IDENTITY, so the POV reticle never moved between scenarios (operator:
    // "each playback seems to show the same point of view"); and the 3D FOV
    // pyramid drew the wrong cone. Training was never affected — the pose is
    // dmp-only and never an NN input — but every downstream analysis of where
    // the camera was pointing was wrong.
    last_camera_view_.camera_pose_world_orient =
        chaseState.getOrientation() * rule_cfg.camera.mount_orientation_body;
    // 040 T029 — FOV is DERIVED from the sensor grid (FR-003), so the dmp
    // records the derived value; there is no separately-configured field that
    // could disagree with the grid it was rendered from.
    last_camera_view_.camera_fov_h_deg =
        static_cast<float>(init.cameraConfig.fovHDeg());   // raw-ok: cereal byte-format member
    last_camera_view_.camera_fov_v_deg =
        static_cast<float>(init.cameraConfig.fovVDeg());   // raw-ok: cereal byte-format member
    last_camera_view_.beacon_left = left;
    last_camera_view_.beacon_right = right;

    last_target_sample_.position = target.position;
    last_target_sample_.orientation = target.orientation;
    last_target_sample_.velocity = target.velocity;
    last_target_sample_.trail_rabbit_position =
        computeTrailRabbit(target, init.trailDistance);
    last_target_sample_.inside_crash_hull =
        isInsideHull(crash_hull_, chaseState.getPosition(), target.position);
}

CrashReason CrrcsimTrackerHelper::tick(AircraftState& chaseState,
                                       NNControllerBackend& nn,
                                       const WorkerInit& init,
                                       gp_scalar pCrashThisGen) {
    if (source_ == nullptr || cursor_ >= source_->samples.size()) {
        return CrashReason::TimeLimit;
    }

    CrashReason crash = CrashReason::None;
    const SourceTickSample& target = source_->samples[cursor_];

    // Step 1: project beacons + shift history (also populates last_camera_view_
    // + last_target_sample_ for M2 dmp recording).
    projectAndShiftHistory(target, chaseState, init);

    // Step 1b (038 P0-D FR-P0H): advance situational-awareness state from the
    // freshly-projected "now" beacon observation. Visibility uses the sentinel
    // threshold. Single-sourced update rule mirrored in TrackerStepper::stepOnce.
    autoc::eval::advanceSituationalAwareness(history_, sa_state_);

    // Step 2: gather tracker NN inputs.
    TrackerInputs inputs = {};
    gather_tracker_inputs(chaseState, history_, init.flightArena,
                          static_cast<float>(init.cepGateThreshold), sa_state_, inputs);

    // Step 3: NN forward pass → updates chaseState.pitch/roll/throttle commands
    // (which inputdev_autoc.cpp's pending-command stage picks up post-tick).
    nn.evaluateTracker(chaseState, inputs);

    // Step 4: arena out-of-bounds via FR-016 FlightArena (same source-of-truth
    // as gather_tracker_inputs slot 44).
    {
        const ArenaEgressKind egress = checkArenaBounds(chaseState, init.flightArena);
        if (egress != ArenaEgressKind::NONE) {
            crash = arenaEgressToCrashReason(egress);
        }
    }

    // 030 M11.preA.3 (2026-05-10) — Crash-hull RE-ENABLED with fixed
    // Bernoulli probability per NN tick (10Hz). 033 cleanup: seed now
    // derives from rabbit-class sub-PRNG via deriveClassSubSeeds (was
    // meta.windSeed pre-cleanup). Mirrors the parallel re-enable in
    // src/eval/tracker_stepper.cc — keep both bodies in lockstep.
    if (crash == CrashReason::None) {
        if (didCrashFire(crash_hull_, chaseState.getPosition(), target.position,
                         pCrashThisGen, prng_state_)) {
            crash = CrashReason::HullStrike;
            ++hull_fired_count_;
        }
    }

    ++cursor_;
    return crash;
}
