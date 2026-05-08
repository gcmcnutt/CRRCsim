// 030 M11.preA — CRRCSim tracker-mode helper class.
//
// Mirror of minisim's TrackerStepper at the per-tick logic level, adapted
// for crrcsim's mod_inputdev calling convention. Encapsulates the source-
// trajectory cursor, beacon history window, crash hull state, beacon
// projection per tick, gather_tracker_inputs, and evaluateTracker NN forward.
//
// Design choices (per specs/030-tracker-mode/m11_prea_plan.md):
// - Helper class, NOT a strategy interface — keeps pathgen body in
//   inputdev_autoc.cpp strictly untouched (regression-tight bitwise gate)
// - Chase init NOT shifted by trail_distance (unlike minisim M9.preB) —
//   crrcsim FDM defaults hb1 to ~13 m/s cruise, no need to defeat the
//   minisim 20-m/s init spike
// - All per-tick primitives reused from autoc_common: projectBeacon,
//   gather_tracker_inputs, computeTrailRabbit (via the tick body),
//   CrashHull, FlightArena
// - Determinism: zero wall-clock / thread-id state. PRNG seeded from
//   scenarioMetadata.scenarioSequence (already deterministic in autoc).

#pragma once

#include <cstdint>

#include "autoc/eval/aircraft_state.h"
#include "autoc/eval/beacon_config.h"
#include "autoc/eval/camera_config.h"
#include "autoc/eval/camera_projection.h"
#include "autoc/eval/crash_hull.h"
#include "autoc/eval/source_trajectory.h"
#include "autoc/nn/evaluator.h"
#include "autoc/rpc/protocol.h"

class CrrcsimTrackerHelper {
public:
    // Called at scenario boundary (analogous to TrackerStepper::initScenario):
    // - Cursor = 0
    // - PRNG seeded from scenarioMetadata.scenarioSequence (with anti-zero guard)
    // - Crash hull initialized from EvalData
    // - History window pre-filled with source[0] projection replicated 6× so
    //   the NN sees a coherent stationary-source history at first tick
    //
    // chaseState is FDM-initialized by the caller before this is called
    // (the M1-style scenario reset path in inputdev_autoc.cpp populates
    // initial chase pose from the FDM post-reset state).
    void initScenario(const SourceScenarioTrajectory& source,
                      const ScenarioMetadata& meta,
                      const EvalData& evalData,
                      AircraftState& chaseState,
                      NNControllerBackend& nn);

    // Per-NN-tick body. chaseState is post-FDM-step (caller has just
    // updated it from FDM). Returns:
    //   - CrashReason::HullStrike if didCrashFire fires this tick
    //   - CrashReason::Eval if checkArenaBounds reports egress
    //   - CrashReason::None otherwise (caller adds time-limit / source-end
    //     checks separately, matching minisim TrackerStepper convention)
    //
    // Side effects:
    //   - cursor_ advances
    //   - history_ shifts left + slot 5 = new beacon projection
    //   - last_camera_view_ + last_target_sample_ populated for M2 dmp
    //   - chaseState.pitchCommand/rollCommand/throttleCommand updated by NN
    CrashReason tick(AircraftState& chaseState,
                     NNControllerBackend& nn,
                     const EvalData& evalData);

    // For M2 dmp recording — caller pushes these into
    // evalResults.cameraViewList[scenario] / targetTrajectoryList[scenario]
    // per tick, parallel to aircraftStateList push.
    const CameraViewSample& lastCameraView() const { return last_camera_view_; }
    const CopiedTargetSample& lastTargetSample() const { return last_target_sample_; }
    int hullFiredCount() const { return hull_fired_count_; }
    bool sourceExhausted() const { return source_ == nullptr || cursor_ >= source_->samples.size(); }

private:
    // Project both wingtip beacons against the current target sample,
    // shift history left, write slot 5. Also populates last_camera_view_ +
    // last_target_sample_ for M2 dmp output.
    void projectAndShiftHistory(const SourceTickSample& target,
                                const AircraftState& chaseState,
                                const EvalData& evalData);

    size_t cursor_ = 0;
    TrackerHistoryWindow history_{};
    autoc::eval::CrashHull crash_hull_{};
    uint32_t prng_state_ = 0;
    int hull_fired_count_ = 0;
    CameraViewSample last_camera_view_{};
    CopiedTargetSample last_target_sample_{};
    // Reference to source samples — set in initScenario. NOT owned.
    const SourceScenarioTrajectory* source_ = nullptr;
};
