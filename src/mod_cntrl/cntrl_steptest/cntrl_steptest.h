// 043 T046 — control-responsiveness sanity test.
//
// WHAT IT IS: a controller that OVERRIDES the stick with a scripted impulse
// schedule and writes one CSV row per FDM step. It is the sim-side counterpart
// of the 2026-09-07 in-flight step test (specs/043-acro-dual-loop/actuator-pin.md),
// built so the SAME analysis script runs on both sides.
//
// ⭐ WHY A CONTROLLER, and not an FDM change or a new eval mode: Controller::Calc
// already receives (dt, fdm, inputsFromUser, inputsToFDM) EVERY FDM step — 3 ms,
// 333 Hz, better than the 59 Hz the real measurement had. So one self-contained
// class can both GENERATE the input and RECORD the response. No FDM edit, no
// MAX_TRACE_STEPS change, no touching the autoc<->crrcsim eval protocol.
//
// ⭐ AND the controller CHAIN decides what is under test, for free:
//   steptest alone                  -> raw surface response  == the MANUAL flight
//   steptest THEN InavFwRate        -> through the ACRO rate loop
// Declare the order in the model XML; no code change to switch.
//
// ⚠️ This is a TEST controller. It ignores the pilot/NN and flies the schedule.
// Never leave it in a model used for training or for a real eval.
#ifndef CNTRL_STEPTEST_H
#define CNTRL_STEPTEST_H

#include "../controller.h"
#include "../../mod_misc/SimpleXMLTransfer.h"
#include <fstream>
#include <string>
#include <vector>

class Cntrl_StepTest : public Controller
{
public:
  explicit Cntrl_StepTest(SimpleXMLTransfer* cfg);
  ~Cntrl_StepTest() override;

  void Calc(double dt, FDMBase* fdm,
            TSimInputs* pInputsFromUser, TSimInputs* pInputsToFDM) override;
  void Reset() override;

private:
  // One cell of the test matrix: settle at a power/attitude, then pulse an axis.
  struct Cell {
    double throttle;      // [0,1]   power -> sets the trimmed airspeed (the LOAD)
    double elevatorTrim;  // [-0.5,0.5] held through settle -> climb/descent attitude
    int    axis;          // 0 = roll (aileron), 1 = pitch (elevator)
    double amplitude;     // surface units, [-0.5,0.5]; sign gives direction
  };

  // ⚠️ The real steps were a PILOT RAMP (~85 ms), not ideal steps. Matching that
  // ramp is deliberate: an ideal step would make the sim look artificially fast
  // against a measurement that never saw one.
  double rampSec_    = 0.085;
  double settleSec_  = 3.0;   // reach steady flight at this power/attitude
  double holdSec_    = 0.8;   // long enough to see the short period ring out
  double recoverSec_ = 1.5;

  std::vector<Cell> cells_;
  size_t cellIdx_    = 0;
  double tCell_      = 0.0;   // seconds inside the current cell
  double tTotal_     = 0.0;
  bool   done_       = false;

  std::ofstream csv_;
  std::string   csvPath_;
  void writeHeader();
};

#endif
