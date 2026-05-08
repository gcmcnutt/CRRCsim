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
                                        const EvalData& evalData,
                                        AircraftState& chaseState,
                                        NNControllerBackend& nn) {
    source_ = &source;
    cursor_ = 0;
    hull_fired_count_ = 0;

    // CrashHull config from EvalData (autoc-side reads autoc-tracker.ini).
    crash_hull_ = CrashHull{};
    crash_hull_.sphere_radius_m = evalData.crashHullRadius;

    // PRNG seed = scenarioSequence with anti-zero guard. Park-Miller LCG
    // requires non-zero in [1, 2^31-2]. Identical guard logic as minisim's
    // TrackerStepper ctor — same scenario gives same PRNG sequence across
    // crrcsim/minisim (determinism contract).
    const uint32_t seed = static_cast<uint32_t>(meta.scenarioSequence);
    prng_state_ = ((seed == 0 ? 0xC0FFEEu : seed % 0x7FFFFFFFu)) | 1u;

    // Reset NN recurrent state at scenario start (no-op for feedforward).
    nn.reset();

    // Pre-fill history with source[0] projection replicated 6×, so the NN
    // sees a coherent stationary-source history at first tick. Mirrors the
    // M9.preB minisim TrackerStepper init for the pre_roll == 0 case.
    if (!source_->samples.empty()) {
        const SourceTickSample& first = source_->samples.front();
        for (int i = 0; i < 6; ++i) {
            projectAndShiftHistory(first, chaseState, evalData);
        }
    }
    // Cursor stays at 0 — first tick() consumes source[0] (the same sample
    // we just used to pre-fill history; that's fine, history shifts cleanly).
}

void CrrcsimTrackerHelper::projectAndShiftHistory(const SourceTickSample& target,
                                                  const AircraftState& chaseState,
                                                  const EvalData& evalData) {
    // Shift slots [1..5] → [0..4]; new "now" lands at index 5.
    for (int i = 0; i < 5; ++i) {
        history_.left_x[i] = history_.left_x[i + 1];      // raw-ok: NN-byte-format primitive
        history_.left_y[i] = history_.left_y[i + 1];      // raw-ok: NN-byte-format primitive
        history_.left_cep[i] = history_.left_cep[i + 1];  // raw-ok: NN-byte-format primitive
        history_.right_x[i] = history_.right_x[i + 1];    // raw-ok: NN-byte-format primitive
        history_.right_y[i] = history_.right_y[i + 1];    // raw-ok: NN-byte-format primitive
        history_.right_cep[i] = history_.right_cep[i + 1];// raw-ok: NN-byte-format primitive
    }

    // Build projection inputs. Cameras + beacons all come from EvalData
    // (autoc-side parses autoc-tracker.ini and ships these per job).
    ProjectionInput proj;
    proj.chase_position_world = chaseState.getPosition();
    proj.chase_orientation_world = chaseState.getOrientation();
    proj.target_position_world = target.position;
    proj.target_orientation_world = target.orientation;
    proj.camera_mount_chase_body = evalData.cameraConfig.mount_offset_body;
    proj.camera_orientation_chase_body = evalData.cameraConfig.mount_orientation_body;
    proj.camera = evalData.cameraConfig;
    proj.chase_airframe = evalData.airframeProxy;

    // Left beacon.
    proj.beacon_mount_target_body = evalData.beaconLeftConfig.mount_body;
    proj.beacon_emission_axis_target_body = evalData.beaconLeftConfig.emission_axis_body;
    proj.beacon = evalData.beaconLeftConfig;
    BeaconObservation left = projectBeacon(proj);

    // Right beacon.
    proj.beacon_mount_target_body = evalData.beaconRightConfig.mount_body;
    proj.beacon_emission_axis_target_body = evalData.beaconRightConfig.emission_axis_body;
    proj.beacon = evalData.beaconRightConfig;
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
        evalData.cameraConfig.mount_offset_body;
    last_camera_view_.camera_pose_world_orient =
        chaseState.getOrientation() * evalData.cameraConfig.mount_orientation_body;
    last_camera_view_.camera_fov_h_deg =
        static_cast<float>(evalData.cameraConfig.fov_h_deg);   // raw-ok: cereal byte-format member
    last_camera_view_.camera_fov_v_deg =
        static_cast<float>(evalData.cameraConfig.fov_v_deg);   // raw-ok: cereal byte-format member
    last_camera_view_.beacon_left = left;
    last_camera_view_.beacon_right = right;

    last_target_sample_.position = target.position;
    last_target_sample_.orientation = target.orientation;
    last_target_sample_.velocity = target.velocity;
    last_target_sample_.trail_rabbit_position =
        computeTrailRabbit(target, evalData.trailDistance);
    last_target_sample_.inside_crash_hull =
        isInsideHull(crash_hull_, chaseState.getPosition(), target.position);
}

CrashReason CrrcsimTrackerHelper::tick(AircraftState& chaseState,
                                       NNControllerBackend& nn,
                                       const EvalData& evalData) {
    if (source_ == nullptr || cursor_ >= source_->samples.size()) {
        return CrashReason::TimeLimit;
    }

    CrashReason crash = CrashReason::None;
    const SourceTickSample& target = source_->samples[cursor_];

    // Step 1: project beacons + shift history (also populates last_camera_view_
    // + last_target_sample_ for M2 dmp recording).
    projectAndShiftHistory(target, chaseState, evalData);

    // Step 2: gather tracker NN inputs.
    TrackerInputs inputs = {};
    gather_tracker_inputs(chaseState, history_, evalData.flightArena, inputs);

    // Step 3: NN forward pass → updates chaseState.pitch/roll/throttle commands
    // (which inputdev_autoc.cpp's pending-command stage picks up post-tick).
    nn.evaluateTracker(chaseState, inputs);

    // Step 4: arena out-of-bounds via FR-016 FlightArena (same source-of-truth
    // as gather_tracker_inputs slot 44).
    {
        const ArenaEgressKind egress = checkArenaBounds(chaseState, evalData.flightArena);
        if (egress != ArenaEgressKind::NONE) {
            crash = arenaEgressToCrashReason(egress);
        }
    }

    // Step 5: crash hull strike (didCrashFire short-circuits when chase
    // is outside hull — no PRNG draw consumed, scenario stream stays
    // deterministic across hull-miss ticks). Arena egress wins on tie.
    if (crash == CrashReason::None) {
        if (didCrashFire(crash_hull_, chaseState.getPosition(), target.position,
                         evalData.pCrashThisGen, prng_state_)) {
            crash = CrashReason::HullStrike;
            ++hull_fired_count_;
        }
    }

    ++cursor_;
    return crash;
}
