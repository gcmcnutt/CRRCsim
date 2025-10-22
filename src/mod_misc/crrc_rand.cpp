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

namespace {

bool shouldTraceRand() {
  static const bool trace =
      []() {
        const char* env = std::getenv("AUTOC_RNG_TRACE");
        return env != nullptr && env[0] != '\0';
      }();
  return trace;
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

unsigned int CRRC_Random::uRandState16;
unsigned int CRRC_Random::uRandState32;

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
  
  srand(uRandState32);
  if (shouldTraceRand()) {
    randCounter().store(0);
    std::cerr << "[AUTOC:RNG] insertData nData=" << nData
              << " seed=" << uRandState32 << std::endl;
  }
}

void CRRC_Random::reset(unsigned int seed)
{
  uRandState16 = seed & 0x7FFF;
  uRandState32 = seed;
  srand(seed);
  if (shouldTraceRand()) {
    randCounter().store(0);
    std::cerr << "[AUTOC:RNG] reset seed=" << seed
              << " u16=" << uRandState16
              << " u32=" << uRandState32 << std::endl;
  }
}

int CRRC_Random::rand()
{
  int value = ::rand();
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
  phase = 0.0;
}

RandGauss::RandGauss( double sigma, double mean )
{
  SetSigmaAndMean( sigma, mean );
  phase = 0.0;
}

double RandGauss::Get()
{
  double S, Z, U1, U2, V1;
  
  if (phase)
    Z = V2 * fac;
  else
  {
    do
    {
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

  return Z*sigma_ + mean_;  
}
