// 043 T046 — control-responsiveness sanity test. See cntrl_steptest.h.
#include "cntrl_steptest.h"
#include "../../mod_fdm/fdm.h"
#include <cmath>
#include <iostream>
#include <iomanip>

Cntrl_StepTest::Cntrl_StepTest(SimpleXMLTransfer* cfg)
{
  // ⛔ Constitution VII: no silent defaults for anything that changes the
  // measurement. Schedule timings and the matrix must be stated in the XML, so
  // a run is reproducible from the model file alone.
  rampSec_    = cfg->getDouble("rampSec");
  trimSec_    = cfg->getDouble("trimSec");
  trimGain_   = cfg->getDouble("trimGain");
  settleSec_  = cfg->getDouble("settleSec");
  holdSec_    = cfg->getDouble("holdSec");
  recoverSec_ = cfg->getDouble("recoverSec");
  csvPath_    = cfg->getString("csv");

  // Matrix: one <cell .../> per (power, attitude, axis, amplitude).
  for (int i = 0; i < cfg->getChildCount(); i++) {
    SimpleXMLTransfer* c = cfg->getChildAt(i);
    if (c->getName() != "cell") continue;
    Cell cell;
    cell.throttle     = c->getDouble("throttle");
    cell.elevatorTrim = c->getDouble("elevatorTrim");
    cell.axis         = c->getInt("axis");
    cell.amplitude    = c->getDouble("amplitude");
    cells_.push_back(cell);
  }
  if (cells_.empty()) {
    std::cerr << "[StepTest] ⛔ no <cell> entries — nothing to measure\n";
  }

  csv_.open(csvPath_.c_str());
  if (!csv_) {
    std::cerr << "[StepTest] ⛔ cannot open " << csvPath_ << " — no data will be recorded\n";
  } else {
    writeHeader();
  }
  const double per = trimSec_ + settleSec_ + rampSec_ + holdSec_ + recoverSec_;
  std::cerr << "[StepTest] " << cells_.size() << " cells x " << per << " s = "
            << cells_.size() * per << " s -> " << csvPath_ << std::endl;
}

Cntrl_StepTest::~Cntrl_StepTest()
{
  if (csv_.is_open()) csv_.close();
}

void Cntrl_StepTest::Reset()
{
  cellIdx_ = 0; tCell_ = 0.0; tTotal_ = 0.0; done_ = false;
}

void Cntrl_StepTest::writeHeader()
{
  // Native SI, unconverted — the analysis converts, so the raw file stays honest.
  // rate_* are rad/s (multiply by 57.2958 for the deg/s the blackbox reports).
  csv_ << "t_s,cell,phase,throttle,elevator_trim,axis,amplitude,"
       << "cmd_aileron,cmd_elevator,cmd_throttle,elev_trimmed,"
       << "rate_p,rate_q,rate_r,"
       << "phi,theta,psi,"
       << "vel_x,vel_y,vel_z,v_rel_airmass,"
       << "pos_x,pos_y,pos_z,flight_cl\n";
}

void Cntrl_StepTest::Calc(double dt, FDMBase* fdm,
                          TSimInputs* pInputsFromUser, TSimInputs* pInputsToFDM)
{
  (void)pInputsFromUser;   // ⚠️ deliberately ignored: the schedule flies, not the pilot
  if (done_ || cells_.empty()) return;

  const Cell& c = cells_[cellIdx_];
  const double tTrimEnd  = trimSec_;
  const double tSettleEnd = tTrimEnd + settleSec_;
  const double tRampEnd  = tSettleEnd + rampSec_;
  const double tHoldEnd  = tRampEnd + holdSec_;
  const double tCellEnd  = tHoldEnd + recoverSec_;

  // ⭐ AUTO-TRIM phase: integrate elevator against vertical speed until the
  // aircraft holds altitude, then freeze. getVel()(2) is z-DOWN, so positive
  // means descending and the elevator must move nose-up.
  double frac = 0.0;
  const char* phase = "trim";
  if (tCell_ < tTrimEnd) {
    // ⛔ SIGN: crrcsim's elevator is INVERTED — cntrl_inavfwrate carries
    // `pitchCmd = -2*elevator`, so NEGATIVE elevator is NOSE UP. To arrest a
    // descent the trim must therefore go NEGATIVE. The first version added
    // instead of subtracting, so the integrator drove nose-DOWN while the
    // aircraft sank, wound to the clamp, and flew it into the ground. Measured
    // symptom: 5-12 m/s of descent still present at the end of a 4 s trim.
    const double w = fdm->getVel()(2);          // ft/s, +down
    elevTrimmed_ -= trimGain_ * w * dt;
    // ⛔ TRIM HAS PRIORITY OVER AMPLITUDE. An earlier version clamped the trim
    // datum to (0.5 - |amplitude|) so the pulse could never clip; at amplitude
    // 0.35 that left only 0.15 of trim authority, which is NOT enough to hold
    // level -- the aircraft pitched to -88 deg and hit the ground during the
    // SETTLE phase, 4.7 s in, before the test began. Failing to trim is fatal;
    // clipping the pulse is merely a measurement caveat. So trim uses the full
    // surface, and the AMPLITUDE is reduced to whatever room is left (below).
    if (elevTrimmed_ >  0.45) elevTrimmed_ =  0.45;
    if (elevTrimmed_ < -0.45) elevTrimmed_ = -0.45;
    frac = 0.0; phase = "trim";
  } else if (tCell_ < tSettleEnd) {
    frac = 0.0; phase = "settle";
  } else if (tCell_ < tRampEnd) {
    frac = (tCell_ - tSettleEnd) / rampSec_;   // the pilot-ramp match
    phase = "ramp";
  } else if (tCell_ < tHoldEnd) {
    frac = 1.0; phase = "hold";
  } else {
    frac = 0.0; phase = "recover";
  }

  // Trim datum, the cell's attitude OFFSET, then the pulse on the axis.
  double ail = 0.0;
  double ele = elevTrimmed_ + c.elevatorTrim;
  // Reduce the pulse to the surface room the trim left, so it stays a CLEAN
  // step rather than a clipped one. The analysis normalises by the amplitude
  // actually applied (it is in the CSV), so a reduced pulse is still valid data
  // -- just a smaller excitation, which is reported rather than hidden.
  double amp = c.amplitude;
  if (c.axis == 1) {
    const double room = 0.5 - std::fabs(ele);
    if (std::fabs(amp) > room) {
      amp = (amp > 0 ? room : -room);
      static double lastWarn = -1e9;
      if (tTotal_ - lastWarn > 5.0) {
        lastWarn = tTotal_;
        std::cerr << "[StepTest] ⚠️ cell " << cellIdx_ << ": trim " << ele
                  << " leaves only " << room << " of surface; pulse reduced from "
                  << c.amplitude << " to " << amp << " (still a clean step)\n";
      }
    }
  }
  if (c.axis == 0) ail += frac * amp;
  else             ele += frac * amp;

  // Controller::Limit clamps to +-0.5 IN PLACE and reports whether it bit.
  float ailf = static_cast<float>(ail);
  float elef = static_cast<float>(ele);
  if (Limit(ailf) || Limit(elef)) {
    static bool warned = false;
    if (!warned) {
      warned = true;
      std::cerr << "[StepTest] ⚠️ command clipped at the +-0.5 surface limit — "
                   "trim + amplitude exceeds full throw; the measured gain for that "
                   "cell is NOT a clean step. Reduce elevatorTrim or amplitude.\n";
    }
  }
  pInputsToFDM->aileron  = ailf;
  pInputsToFDM->elevator = elef;
  pInputsToFDM->rudder   = 0.0f;              // FR-018: this airframe commands no yaw
  pInputsToFDM->throttle = static_cast<float>(c.throttle);

  if (csv_.is_open()) {
    CRRCMath::Vector3 pqr = fdm->getPQR();
    CRRCMath::Vector3 vel = fdm->getVel();
    CRRCMath::Vector3 pos = fdm->getPos();
    csv_ << std::fixed << std::setprecision(6)
         << tTotal_ << ',' << cellIdx_ << ',' << phase << ','
         << c.throttle << ',' << c.elevatorTrim << ',' << c.axis << ',' << c.amplitude << ','
         << pInputsToFDM->aileron << ',' << pInputsToFDM->elevator << ',' << pInputsToFDM->throttle << ','
         << elevTrimmed_ << ','
         << pqr(0) << ',' << pqr(1) << ',' << pqr(2) << ','
         << fdm->getPhi() << ',' << fdm->getTheta() << ',' << fdm->getPsi() << ','
         << vel(0) << ',' << vel(1) << ',' << vel(2) << ',' << fdm->getVRelAirmass() << ','
         << pos(0) << ',' << pos(1) << ',' << pos(2) << ',' << fdm->getFlightCL() << '\n';
  }

  tCell_  += dt;
  tTotal_ += dt;
  if (tCell_ >= tCellEnd) {
    tCell_ = 0.0;
    elevTrimmed_ = 0.0;   // each cell re-trims from scratch: its throttle differs
    if (++cellIdx_ >= cells_.size()) {
      done_ = true;
      csv_.flush();
      std::cerr << "[StepTest] ✅ schedule complete, " << tTotal_ << " s -> " << csvPath_ << std::endl;
    }
  }
}
