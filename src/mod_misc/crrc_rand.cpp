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

#include <cmath>

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
      U1 = (double)rand() / RAND_MAX;
      U2 = (double)rand() / RAND_MAX;

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

