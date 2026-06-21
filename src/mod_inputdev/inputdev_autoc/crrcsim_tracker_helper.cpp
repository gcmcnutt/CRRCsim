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
#include "autoc/eval/derived_features.h"  // 032 phase 1 — compute_pair_span
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

    // 037 T022 — fail loud on a source library whose tick spacing does not
    // match the compiled cadence (the caller advances one SIM_TIME_STEP_MSEC
    // of chase physics per source tick; a 100 ms-recorded library at a 50 ms
    // cadence would silently play the target at 2× speed). Mirrors
    // src/eval/tracker_stepper.cc::initScenario.
    // 2026-06-15: check the AVERAGE gap, not the first gap. simTimeMsec is the
    // 200 Hz/5 ms step clock TRUNCATED to integer ms, so a clean 20 Hz/50 ms
    // source records gaps of 49/50/51 (re-syncing to exact 50-multiples) with
    // the FIRST gap deterministically 49 — a single-gap test spuriously rejects
    // every valid source. (last-first)/(N-1) recovers the true cadence exactly
    // (50.0) and still catches a real mismatch (a 100 ms 10 Hz source → 100).
    // Proper fix = round/step-count the simTimeMsec stamp (BACKLOG).
    if (source_->samples.size() >= 2) {
        const auto& s = source_->samples;
        const double avgGapMsec =
            (s.back().simTimeMsec - s.front().simTimeMsec) /
            static_cast<double>(s.size() - 1);
        if (std::lround(avgGapMsec) != SIM_TIME_STEP_MSEC) {
            throw std::runtime_error(
                "CrrcsimTrackerHelper: source trajectory avg tick spacing " +
                std::to_string(avgGapMsec) + " ms != compiled SIM_TIME_STEP_MSEC " +
                std::to_string(SIM_TIME_STEP_MSEC) +
                " ms — rebake the M2 source library at the current cadence.");
        }
    }

    // Pre-fill history with source[0] projection replicated across the WHOLE
    // observation ring (037: depth grew with the R5 lag window), so the NN
    // sees a coherent stationary-source history at first tick. Mirrors the
    // minisim TrackerStepper init for the pre_roll == 0 case.
    obs_ring_.reset();
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

    // 030 V1 priming — cameras + beacons + airframe come from WorkerInit
    // (sent once per worker; autoc-side parses autoc-tracker.ini at startup).
    ProjectionInput proj;
    proj.chase_position_world = chaseState.getPosition();
    proj.chase_orientation_world = chaseState.getOrientation();
    proj.target_position_world = target.position;
    proj.target_orientation_world = target.orientation;
    proj.camera_mount_chase_body = init.cameraConfig.mount_offset_body;
    proj.camera_orientation_chase_body = init.cameraConfig.mount_orientation_body;
    proj.camera = init.cameraConfig;
    proj.chase_airframe = init.airframeProxy;

    // Left beacon.
    proj.beacon_mount_target_body = init.beaconLeftConfig.mount_body;
    proj.beacon_emission_axis_target_body = init.beaconLeftConfig.emission_axis_body;
    proj.beacon = init.beaconLeftConfig;
    BeaconObservation left = projectBeacon(proj);

    // Right beacon.
    proj.beacon_mount_target_body = init.beaconRightConfig.mount_body;
    proj.beacon_emission_axis_target_body = init.beaconRightConfig.emission_axis_body;
    proj.beacon = init.beaconRightConfig;
    BeaconObservation right = projectBeacon(proj);

    TrackerObservationRing::Record rec;
    rec.left_x = left.screen_x;       // raw-ok: NN-byte-format primitive
    rec.left_y = left.screen_y;       // raw-ok: NN-byte-format primitive
    rec.left_cep = left.cep;          // raw-ok: NN-byte-format primitive
    rec.right_x = right.screen_x;     // raw-ok: NN-byte-format primitive
    rec.right_y = right.screen_y;     // raw-ok: NN-byte-format primitive
    rec.right_cep = right.cep;        // raw-ok: NN-byte-format primitive

    // 032 PHASE 1 — Cache beacon-pair span at the current tick. CEP-gated:
    // if EITHER beacon's CEP exceeds the configured threshold, substitute
    // neutral 0.0. Mirrors src/eval/tracker_stepper.cc::projectAndShiftHistory
    // exactly. cep_gate_threshold comes from init.cepGateThreshold (no
    // ConfigManager on the crrcsim worker — value threaded via WorkerInit).
    const float cep_gate_threshold = static_cast<float>(init.cepGateThreshold);
    const bool cep_gated =
        left.cep >= cep_gate_threshold ||
        right.cep >= cep_gate_threshold;
    if (cep_gated) {
        rec.span = 0.0f;
    } else {
        rec.span = static_cast<float>(  // raw-ok: NN-byte-format slot write
            autoc::eval::compute_pair_span(
                static_cast<gp_scalar>(left.screen_x),
                static_cast<gp_scalar>(left.screen_y),
                static_cast<gp_scalar>(right.screen_x),
                static_cast<gp_scalar>(right.screen_y)));
    }
    obs_ring_.push(rec);
    obs_ring_.materialize(history_);

    // M2 dmp recording — mirror minisim's M8b populate.
    last_camera_view_.camera_pose_world_pos =
        chaseState.getPosition() + chaseState.getOrientation() *
        init.cameraConfig.mount_offset_body;
    last_camera_view_.camera_pose_world_orient =
        chaseState.getOrientation() * init.cameraConfig.mount_orientation_body;
    last_camera_view_.camera_fov_h_deg =
        static_cast<float>(init.cameraConfig.fov_h_deg);   // raw-ok: cereal byte-format member
    last_camera_view_.camera_fov_v_deg =
        static_cast<float>(init.cameraConfig.fov_v_deg);   // raw-ok: cereal byte-format member
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

    // Step 2: gather tracker NN inputs.
    TrackerInputs inputs = {};
    gather_tracker_inputs(chaseState, history_, init.flightArena,
                          static_cast<float>(init.cepGateThreshold), inputs);

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
