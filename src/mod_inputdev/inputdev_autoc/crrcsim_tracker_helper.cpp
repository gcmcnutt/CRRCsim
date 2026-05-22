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

    // 033 §2.B — capture smoothness config from WorkerInit; reset per-tick
    // state. First-tick factor = 1.0 (no false-positive penalty on scenario
    // entry). Mirror of TrackerStepper::initScenario smoothness reset.
    smoothness_floor_ = init.smoothnessPenaltyFloor;
    smoothness_mode_ = init.smoothnessMotionMode;
    prev_out_valid_ = false;
    prev_out_pt_ = static_cast<gp_scalar>(0.0);
    prev_out_rl_ = static_cast<gp_scalar>(0.0);
    prev_out_th_ = static_cast<gp_scalar>(0.0);

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
        history_.span[i] = history_.span[i + 1];          // raw-ok: NN-byte-format primitive — 032 phase 1
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
        history_.span[5] = 0.0f;
    } else {
        history_.span[5] = static_cast<float>(  // raw-ok: NN-byte-format slot write
            autoc::eval::compute_pair_span(
                static_cast<gp_scalar>(left.screen_x),
                static_cast<gp_scalar>(left.screen_y),
                static_cast<gp_scalar>(right.screen_x),
                static_cast<gp_scalar>(right.screen_y)));
    }

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

    // 033 §2.B — per-tick smoothness factor from NN-output Δs vs previous
    // tick; store on chaseState for autoc-side fitness consumption (single
    // source of truth: fitness_decomposition.cc reads getSmoothnessFactor()).
    // Mirrors the autoc-side TrackerStepper::stepOnce smoothness compute.
    // First tick (prev_out_valid_ = false) → Δs all zero → factor = 1.0.
    {
        const gp_scalar curr_pt = chaseState.getPitchCommand();
        const gp_scalar curr_rl = chaseState.getRollCommand();
        const gp_scalar curr_th = chaseState.getThrottleCommand();
        gp_scalar dpt = static_cast<gp_scalar>(0.0);
        gp_scalar drl = static_cast<gp_scalar>(0.0);
        gp_scalar dth = static_cast<gp_scalar>(0.0);
        if (prev_out_valid_) {
            dpt = curr_pt - prev_out_pt_;
            drl = curr_rl - prev_out_rl_;
            dth = curr_th - prev_out_th_;
        }
        // Decode wire-stable uint8 enum → autoc::eval::SmoothnessMotionMode.
        autoc::eval::SmoothnessMotionMode mode;
        switch (smoothness_mode_) {
            case 1: mode = autoc::eval::SmoothnessMotionMode::Sum; break;
            case 2: mode = autoc::eval::SmoothnessMotionMode::Max; break;
            default: mode = autoc::eval::SmoothnessMotionMode::Pythagorean; break;
        }
        const gp_scalar factor = autoc::eval::compute_smoothness_factor(
            dpt, drl, dth, smoothness_floor_, mode);
        chaseState.setSmoothnessFactor(factor);
        prev_out_pt_ = curr_pt;
        prev_out_rl_ = curr_rl;
        prev_out_th_ = curr_th;
        prev_out_valid_ = true;
    }

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
