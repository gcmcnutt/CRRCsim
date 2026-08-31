/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * 043 US2 — Cntrl_InavFwRate implementation. See the header + the pure core
 * include/autoc/control/inav_fw_rate.h and contracts/inav-fw-rate-loop.md.
 */
#include "cntrl_inavfwrate.h"

#include <algorithm>  // std::clamp
#include <cmath>      // M_PI
#include <iostream>   // startup log of the action-space scale

autoc::control::InavFwRateGains
Cntrl_InavFwRate::readAxis(SimpleXMLTransfer* cfg)
{
  // No defaults on getDouble → a missing key fail-louds (Constitution VII). All
  // constants come from the MODEL's <config><controllers> node (FR-014), so they
  // change without a rebuild.
  autoc::control::InavFwRateGains g;
  g.kP                        = cfg->getDouble("kP");
  g.kI                        = cfg->getDouble("kI");
  g.kD                        = cfg->getDouble("kD");
  g.kFF                       = cfg->getDouble("kFF");
  g.maxRate                   = cfg->getDouble("maxRate");
  g.gyroLpfHz                 = cfg->getDouble("gyroLpfHz");
  g.dtermLpfHz                = cfg->getDouble("dtermLpfHz");
  g.itermLockRateThresholdPct = cfg->getDouble("itermLockRateThresholdPct");
  g.engageThresholdPct        = cfg->getDouble("engageThresholdPct");
  g.lockTimeMaxMs             = cfg->getDouble("lockTimeMaxMs");
  g.itermLimitPct             = cfg->getDouble("itermLimitPct");
  g.pidSumLimit               = cfg->getDouble("pidSumLimit");
  return g;
}

Cntrl_InavFwRate::Cntrl_InavFwRate(SimpleXMLTransfer* cfg)
{
  gainsRoll_  = readAxis(cfg->getChild("roll"));
  gainsPitch_ = readAxis(cfg->getChild("pitch"));
  // 043 action-space scale (see header). No default: a missing attribute
  // fail-louds, so every config must state which arm it is (Constitution VII).
  cmdScaleRoll_  = cfg->getChild("roll")->getDouble("commandScale");
  cmdScalePitch_ = cfg->getChild("pitch")->getDouble("commandScale");
  std::cerr << "[InavFwRate] commandScale roll=" << cmdScaleRoll_
            << " pitch=" << cmdScalePitch_
            << "  => full command commands "
            << (cmdScaleRoll_ * gainsRoll_.maxRate) << " / "
            << (cmdScalePitch_ * gainsPitch_.maxRate) << " deg/s"
            << std::endl;
  Reset();
}

void Cntrl_InavFwRate::Reset()
{
  stateRoll_.reset();
  statePitch_.reset();
}

void Cntrl_InavFwRate::Calc(double      dt,
                            FDMBase*    fdm,
                            TSimInputs* pInputsFromUser,
                            TSimInputs* pInputsToFDM)
{
  const double RAD2DEG = 180.0 / M_PI;
  CRRCMath::Vector3 omega = fdm->getPQR();   // body rates (p, q, r), rad/s

  // Recover the NN command ∈ [-1,+1] from the crrcsim stick convention that
  // inputdev_autoc::getInputData writes into pInputsFromUser:
  //   aileron  = +rollCmd / 2      (so rollCmd  =  2·aileron)
  //   elevator = -pitchCmd / 2     (so pitchCmd = -2·elevator; the pitch sign
  //                                 flip is the existing crrcsim convention)
  // 043 action-space scale applied HERE (adapter side, FR-016): full command
  // maps to commandScale x maxRate. At 1.0 this is a bit-exact no-op.
  const double rollCmd  =  2.0 * static_cast<double>(pInputsFromUser->aileron)  * cmdScaleRoll_;
  const double pitchCmd = -2.0 * static_cast<double>(pInputsFromUser->elevator) * cmdScalePitch_;

  const double outRoll  = autoc::control::inavFwRateStep(
      gainsRoll_,  stateRoll_,  dt, rollCmd,  omega(0) * RAD2DEG);
  const double outPitch = autoc::control::inavFwRateStep(
      gainsPitch_, statePitch_, dt, pitchCmd, omega(1) * RAD2DEG);

  // axisPID (±pidSumLimit) → surface (±0.5), mirroring the MANUAL sign
  // convention (aileron +, elevator −) so a given command produces the same
  // surface polarity in both modes. ⭐ This is where the chase becomes
  // RATE-driven rather than surface-driven (contracts/action-space.md).
  pInputsToFDM->aileron  = static_cast<float>(std::clamp(
      outRoll  / gainsRoll_.pidSumLimit  * 0.5, -0.5, 0.5));
  pInputsToFDM->elevator = static_cast<float>(std::clamp(
      -outPitch / gainsPitch_.pidSumLimit * 0.5, -0.5, 0.5));

  // throttle is a DIRECT command, not a rate (FR-017): leave the value
  // ControllerCallback already CopyFrom'd. Yaw reaches no surface (FR-018).
  pInputsToFDM->rudder = 0.0f;
}
