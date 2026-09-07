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
  const double per = settleSec_ + rampSec_ + holdSec_ + recoverSec_;
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
       << "cmd_aileron,cmd_elevator,cmd_throttle,"
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
  const double tRampEnd  = settleSec_ + rampSec_;
  const double tHoldEnd  = tRampEnd + holdSec_;
  const double tCellEnd  = tHoldEnd + recoverSec_;

  // Fraction of the commanded amplitude currently applied.
  double frac = 0.0;
  const char* phase = "settle";
  if (tCell_ < settleSec_) {
    frac = 0.0; phase = "settle";
  } else if (tCell_ < tRampEnd) {
    frac = (tCell_ - settleSec_) / rampSec_;   // the pilot-ramp match
    phase = "ramp";
  } else if (tCell_ < tHoldEnd) {
    frac = 1.0; phase = "hold";
  } else {
    frac = 0.0; phase = "recover";
  }

  // Trim, then the pulse on the axis under test.
  double ail = 0.0;
  double ele = c.elevatorTrim;
  if (c.axis == 0) ail += frac * c.amplitude;
  else             ele += frac * c.amplitude;

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
         << pqr(0) << ',' << pqr(1) << ',' << pqr(2) << ','
         << fdm->getPhi() << ',' << fdm->getTheta() << ',' << fdm->getPsi() << ','
         << vel(0) << ',' << vel(1) << ',' << vel(2) << ',' << fdm->getVRelAirmass() << ','
         << pos(0) << ',' << pos(1) << ',' << pos(2) << ',' << fdm->getFlightCL() << '\n';
  }

  tCell_  += dt;
  tTotal_ += dt;
  if (tCell_ >= tCellEnd) {
    tCell_ = 0.0;
    if (++cellIdx_ >= cells_.size()) {
      done_ = true;
      csv_.flush();
      std::cerr << "[StepTest] ✅ schedule complete, " << tTotal_ << " s -> " << csvPath_ << std::endl;
    }
  }
}
