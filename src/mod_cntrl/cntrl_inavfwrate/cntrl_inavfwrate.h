/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * 043 US2 — Cntrl_InavFwRate: models INAV's fixed-wing ACRO *rate* controller
 * so the chase aircraft is driven by rotation-rate setpoints instead of surface
 * deflections. The loop math is the pure, unit-tested core in
 * include/autoc/control/inav_fw_rate.h (contracts/inav-fw-rate-loop.md); this
 * class is only the crrcsim adapter that wires fdm->getPQR() and the stick
 * command into that core and maps the ±pidSumLimit output onto the surface.
 *
 * ⛔ ACRO = RATE control — NO attitude term (FR-019a). The core has no attitude
 * input at all, so that is structurally guaranteed.
 *
 * Loaded MODEL-LOCALLY from the airplane's own <config><controllers> node
 * (crrcsim/models/hb1_streamer.xml) by fdm_larcsim, following the fdm_mcopter01
 * pattern — the FC tune belongs to the airframe it was tuned for, so another
 * model cannot inherit these gains and per-scenario gain variation stays
 * reachable. It runs per FDM substep BEFORE the 037 servo model, so servo lag
 * lands inside the rate loop.
 */
#ifndef CNTRL_INAVFWRATE_H
#define CNTRL_INAVFWRATE_H

#include "../controller.h"

#include "autoc/control/inav_fw_rate.h"

class Cntrl_InavFwRate : public Controller
{
public:
  Cntrl_InavFwRate(SimpleXMLTransfer* cfg);

  virtual void Reset();

  virtual void Calc(double      dt,
                    FDMBase*    fdm,
                    TSimInputs* pInputsFromUser,
                    TSimInputs* pInputsToFDM);

  virtual ~Cntrl_InavFwRate() {}

private:
  // Per-axis constants (roll, pitch) read from XML with NO in-class fallback
  // (Constitution VII): getDouble(key) throws if a key is missing.
  static autoc::control::InavFwRateGains readAxis(SimpleXMLTransfer* cfg);

  autoc::control::InavFwRateGains gainsRoll_;
  autoc::control::InavFwRateGains gainsPitch_;
  autoc::control::InavFwRateState stateRoll_;
  autoc::control::InavFwRateState statePitch_;

  // 043 — ACTION-SPACE scale: full command (|cmd| = 1) maps to
  // commandScale x maxRate instead of maxRate. This is OURS (FR-016), not an
  // INAV parameter, which is why it lives in the adapter and NOT in the
  // validated inav_fw_rate.h core (that core matches real flight data to
  // r = 0.9999 and is deliberately left alone).
  //
  // WHY IT EXISTS (measured 2026-08-30 against the 041-t7 ACRO flight): the
  // P/D attenuation depends ONLY on the commanded fraction of maxRate,
  // aP = aD = exp(-17.33 f^2), so f = 1 gives aP ~ 0 REGARDLESS of `rates`.
  // The human pilot flies f ~ 0.05-0.11 (aP 0.82-0.91, loop closed and
  // damping); the gen-63 policy pegged at f ~ 0.87-0.90 (aP ~ 0.035, loop
  // degenerate to pure feed-forward). Since 043 exists to let the inner loop
  // damp the 2-5 Hz oscillation, and that damping IS P/D, the policy must be
  // able to reach the regime where P/D survive. 1.0 = pre-043-A/B behaviour
  // (exactly: x1.0 is bit-exact in IEEE754).
  double cmdScaleRoll_;
  double cmdScalePitch_;
};

#endif
