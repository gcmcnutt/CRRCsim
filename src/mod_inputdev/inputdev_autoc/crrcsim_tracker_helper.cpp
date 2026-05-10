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

#include "autoc/eval/arena.h"
#include "autoc/eval/trail_rabbit.h"

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

    // PRNG seed = windSeed with anti-zero guard. Park-Miller LCG requires
    // non-zero in [1, 2^31-2]. windSeed (NOT scenarioSequence) for
    // determinism: scenarioSequence is a monotonic counter that differs
    // between training-eval (seq=N) and elite-reeval (seq=N+~5000+) of the
    // same scenario, which produced different LCG sequences → different
    // didCrashFire Bernoulli draws → ELITE_DIVERGED warnings (2026-05-09).
    // windSeed is per-scenario joint-PRNG-derived, stable across train/
    // elite for the same scenario index. Same scenario → same hull-PRNG.
    const uint32_t seed = static_cast<uint32_t>(meta.windSeed);
    prng_state_ = ((seed == 0 ? 0xC0FFEEu : seed % 0x7FFFFFFFu)) | 1u;

    // Reset NN recurrent state at scenario start (no-op for feedforward).
    nn.reset();

    // Pre-fill history with source[0] projection replicated 6×, so the NN
    // sees a coherent stationary-source history at first tick. Mirrors the
    // M9.preB minisim TrackerStepper init for the pre_roll == 0 case.
    if (!source_->samples.empty()) {
        const SourceTickSample& first = source_->samples.front();
        for (int i = 0; i < 6; ++i) {
            projectAndShiftHistory(first, chaseState, init);
        }
    }
    // Cursor stays at 0 — first tick() consumes source[0] (the same sample
    // we just used to pre-fill history; that's fine, history shifts cleanly).
}

void CrrcsimTrackerHelper::projectAndShiftHistory(const SourceTickSample& target,
                                                  const AircraftState& chaseState,
                                                  const WorkerInit& init) {
    // Shift slots [1..5] → [0..4]; new "now" lands at index 5.
    for (int i = 0; i < 5; ++i) {
        history_.left_x[i] = history_.left_x[i + 1];      // raw-ok: NN-byte-format primitive
        history_.left_y[i] = history_.left_y[i + 1];      // raw-ok: NN-byte-format primitive
        history_.left_cep[i] = history_.left_cep[i + 1];  // raw-ok: NN-byte-format primitive
        history_.right_x[i] = history_.right_x[i + 1];    // raw-ok: NN-byte-format primitive
        history_.right_y[i] = history_.right_y[i + 1];    // raw-ok: NN-byte-format primitive
        history_.right_cep[i] = history_.right_cep[i + 1];// raw-ok: NN-byte-format primitive
    }

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

    history_.left_x[5] = left.screen_x;       // raw-ok: NN-byte-format primitive
    history_.left_y[5] = left.screen_y;       // raw-ok: NN-byte-format primitive
    history_.left_cep[5] = left.cep;          // raw-ok: NN-byte-format primitive
    history_.right_x[5] = right.screen_x;     // raw-ok: NN-byte-format primitive
    history_.right_y[5] = right.screen_y;     // raw-ok: NN-byte-format primitive
    history_.right_cep[5] = right.cep;        // raw-ok: NN-byte-format primitive

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
    gather_tracker_inputs(chaseState, history_, init.flightArena, inputs);

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
    // Bernoulli probability per NN tick (10Hz). Seed = windSeed (P1 fix,
    // stable train↔elite). Mirrors the parallel re-enable in
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
