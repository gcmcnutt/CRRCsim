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
 * Loaded from the GLOBAL config's <controllers> node (crrc_fdm.cpp reads the
 * global cfg, model-independent — NOT the model XML; see data-model.md §3). It
 * runs per FDM substep BEFORE the 037 servo model, so servo lag lands inside the
 * rate loop.
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
};

#endif
