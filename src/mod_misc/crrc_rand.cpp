/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2006, 2008 Jens Wilhelm Wulf (original author)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#include "crrc_rand.h"

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>

namespace {

struct RandGaussEvent {
  double simTimeMs;
  void* objectPtr;
  int phaseBefore;
  int phaseAfter;
  int loopCount;
  double S;
  uint32_t rng32After;
};

// Thread-local event buffer for ordered logging
thread_local std::vector<RandGaussEvent>* gEventBuffer = nullptr;
thread_local int gEventLimit = 0;

bool shouldTraceRand() {
  static const bool envEnabled =
      []() {
        const char* env = std::getenv("AUTOC_RNG_TRACE");
        return env != nullptr && env[0] != '\0';
      }();
  // Only trace when both env var is set AND we're in deterministic test mode
  return envEnabled;
}

std::atomic<uint64_t>& randCounter() {
  static std::atomic<uint64_t> counter{0};
  return counter;
}

thread_local const char* gRandContext = nullptr;

double uniformUnit(const char* label) {
  const char* previous = CRRC_Random::pushTraceContext(label);
  int raw = CRRC_Random::rand();
  CRRC_Random::popTraceContext(previous);
  return static_cast<double>(raw) / CRRC_Random::max();
}

}  // namespace

// Initialize RNG state variables to safe defaults
// uRandState32 = 1 is the standard seed for Park-Miller LCG
unsigned int CRRC_Random::uRandState16 = 0;
unsigned int CRRC_Random::uRandState32 = 1;

void CRRC_Random::insertData(int nData)
{
  uRandState16 += nData;
  
  const int a = 1103515245;
  const int c = 12345;
  uRandState16 = (a * uRandState16 + c) % 0x7FFF;
  
  uRandState32 = (uRandState32 << 5) ^ (uRandState16 << 3) ^ nData;
  
  //  std::cout << uRandState32 << "\n";
  // Histogramm of uRandState32, test with 106557 values:
  // Only two values occured twice, all the others only once.
  // Groups showed good distribution:
  //    1000:   most once, lots two times, 10 three times.
  //    2000:   like 1000, but more three times, one four times.
  //    5000:   everything fine
  //   50000:   everything fine
  //  500000:   everything fine
  // Value over time is equally distributed, too.

  // Note: srand() removed - we use Park-Miller LCG instead of system rand()
  if (shouldTraceRand()) {
    randCounter().store(0);
    std::cerr << "[AUTOC:RNG] insertData nData=" << nData
              << " seed=" << uRandState32 << std::endl;
  }
}

void CRRC_Random::reset(unsigned int seed)
{
  uRandState16 = seed & 0x7FFF;
  // Ensure seed is in valid range [1, 2^31-1] for Park-Miller LCG
  uRandState32 = (seed == 0) ? 1 : (seed & 0x7FFFFFFF);
  if (uRandState32 == 0) uRandState32 = 1;
  // Note: srand() removed - we use Park-Miller LCG instead of system rand()
  if (shouldTraceRand()) {
    randCounter().store(0);
    std::cerr << "[AUTOC:RNG] reset seed=" << seed
              << " u16=" << uRandState16
              << " u32=" << uRandState32 << std::endl;
  }
}

int CRRC_Random::rand()
{
  // Use Park-Miller LCG for deterministic cross-platform RNG
  // This ensures identical behavior regardless of platform rand() implementation
  const uint64_t a = 48271;  // Park-Miller multiplier
  const uint64_t m = 0x7FFFFFFF;  // 2^31 - 1 (Mersenne prime)

  // Ensure state is never 0 (would cause degenerate sequence)
  if (uRandState32 == 0) uRandState32 = 1;

  // Linear congruential generator: state = (a * state) % m
  uRandState32 = static_cast<unsigned int>((a * uRandState32) % m);

  int value = static_cast<int>(uRandState32);

  if (shouldTraceRand()) {
    auto& counter = randCounter();
    uint64_t callIndex = ++counter;
    std::cerr << "[AUTOC:RNG] rand call=" << callIndex
              << " value=" << value;
    if (gRandContext && gRandContext[0] != '\0') {
      std::cerr << " context=" << gRandContext;
    }
    std::cerr << std::endl;
  }
  return value;
}

const char* CRRC_Random::pushTraceContext(const char* context)
{
  const char* previous = gRandContext;
  gRandContext = context;
  return previous;
}

void CRRC_Random::popTraceContext(const char* previousContext)
{
  gRandContext = previousContext;
}

RandGauss::RandGauss()
{
  SetSigmaAndMean( 1.0, 0.0 );
  phase = 0;
  V2 = 0.0;
  fac = 0.0;
}

RandGauss::RandGauss( double sigma, double mean )
{
  SetSigmaAndMean( sigma, mean );
  phase = 0;
  V2 = 0.0;
  fac = 0.0;
}

void RandGauss::Reset()
{
  phase = 0;
  V2 = 0.0;
  fac = 0.0;
}

double RandGauss::Get()
{
  extern double gSubFrameTime;  // Simulation time in seconds

  int phaseBefore = phase;
  double S = 0.0, Z, U1, U2, V1;
  int loopCount = 0;

  if (phase)
    Z = V2 * fac;
  else
  {
    do
    {
      loopCount++;
      U1 = uniformUnit("RandGauss::Get.U1");
      U2 = uniformUnit("RandGauss::Get.U2");

      V1 = 2 * U1 - 1;
      V2 = 2 * U2 - 1;
      S = V1 * V1 + V2 * V2;
    }
    while(S >= 1);

    fac = sqrt (-2 * log(S) / S);
    Z = V1 * fac;
  }

  phase = 1 - phase;

  // Log event if buffer is active
  if (gEventBuffer && static_cast<int>(gEventBuffer->size()) < gEventLimit) {
    double simTimeMs = gSubFrameTime * 1000.0;
    if (simTimeMs >= 30.0 && simTimeMs <= 45.0) {
      gEventBuffer->push_back(RandGaussEvent{
        simTimeMs,
        this,
        phaseBefore,
        phase,
        loopCount,
        S,
        CRRC_Random::getState32()
      });
    }
  }

  return Z*sigma_ + mean_;
}

// Public API for ordered event logging
namespace RandGaussTrace {
  void startEventLog(int maxEvents) {
    if (!gEventBuffer) {
      gEventBuffer = new std::vector<RandGaussEvent>();
    }
    gEventBuffer->clear();
    gEventLimit = maxEvents;
  }

  void dumpAndClearEventLog() {
    if (!gEventBuffer) return;
    
    for (const auto& ev : *gEventBuffer) {
      std::cerr << "RandGauss::Get() t=" << ev.simTimeMs
                << "ms this=" << ev.objectPtr
                << " phase:" << ev.phaseBefore << "->" << ev.phaseAfter;
      if (ev.phaseBefore == 0) {
        std::cerr << " loops=" << ev.loopCount << " S=" << ev.S;
      }
      std::cerr << " rng32=" << ev.rng32After << std::endl;
    }
    gEventBuffer->clear();
  }

  void stopEventLog() {
    if (gEventBuffer) {
      delete gEventBuffer;
      gEventBuffer = nullptr;
    }
    gEventLimit = 0;
  }
}

