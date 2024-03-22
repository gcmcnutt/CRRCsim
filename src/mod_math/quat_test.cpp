/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2008-2009 - Jens Wilhelm Wulf (original author)
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
#include <iostream>

#include "quaternion.h"
#include "vector3.h"
#include "matrix33.h"
#include "intgr.h"

/**
 * Quaternion test.
 * 
 * velocity = sin(x)*sin(x)
 * angle    = integral(velocity) dt
 *          = integral(sin(x)*sin(x)) dt
 *          = -0.5 cos(x) sin(x) + 0.5 x
 *              
 */
int main()
{
  const int                   nSteps = 20;
  CRRCMath::Quaternion_002    q1;
  double                      dt = M_PI/((double)nSteps);
  double                      val;
  double                      t;
  double                      soll;
  double                      soll_ende = 0;
  double                      soll_ende_tmp;
  int                         zeit = 0;
  CRRCMath::Vector3           achse[3];
  double                      soll_mod;

  achse[0] = CRRCMath::Vector3(1, 0, 0);
  achse[1] = CRRCMath::Vector3(0, 1, 0);
  achse[2] = CRRCMath::Vector3(0, 0, 1);

  q1.init(CRRCMath::Vector3());


  std::cout << "#N time reference_value phi theta psi l2b[0|0] l2b[0|1] l2b[0|2] l2b[1|0] l2b[1|1] l2b[1|2] l2b[2|0] l2b[2|1] l2b[2|2]\n";
  std::cout << "#M 999999 9 9 9 9 9 9 9 9 9 9 9 9 9\n";
  
  for (int a=0; a<3; a++)
  {
    for (int c=0; c<8; c++)
    {
      for (int n=1; n<=nSteps; n++)
      {
        t = M_PI*n/((double)nSteps);

        soll = soll_ende - 0.5*cos(t)*sin(t) + 0.5*t;
        soll_ende_tmp = soll;
        val  = sin(t)*sin(t);

        soll_mod = soll;
        while (soll_mod > M_PI)
          soll_mod -= M_PI;
        while (soll_mod < -M_PI)
          soll_mod += M_PI;
        
        q1.step(dt, achse[a] * val);

        q1.updateEuler();
                
        std::cout << zeit++ << " " << soll_mod << " " << q1.euler.r[0] << " " << q1.euler.r[1] << " " << q1.euler.r[2] << " ";
        q1.mat.printLine(); 
        std::cout << "\n";
      }
      soll_ende = soll_ende_tmp;
    }

    for (int c=0; c<8; c++)
    {
      for (int n=1; n<=nSteps; n++)
      {
        t = M_PI*n/((double)nSteps);

        soll = soll_ende + 0.5*cos(t)*sin(t) - 0.5*t;
        soll_ende_tmp = soll;
        val  = sin(t)*sin(t);

        soll_mod = soll;
        while (soll_mod > M_PI)
          soll_mod -= M_PI;
        while (soll_mod < -M_PI)
          soll_mod += M_PI;

        q1.step(dt, achse[a] * -1 * val);

        q1.updateEuler();

        std::cout << zeit++ << " " << soll_mod << " " << q1.euler.r[0] << " " << q1.euler.r[1] << " " << q1.euler.r[2] << " ";
        q1.mat.printLine(); 
        std::cout << "\n";
      }
      soll_ende = soll_ende_tmp;      
    }
    
    for (int n=0; n<4*nSteps; n++)
    {
      std::cout << zeit++ << " " << soll_mod << " " << q1.euler.r[0] << " " << q1.euler.r[1] << " " << q1.euler.r[2] << " ";
      q1.mat.printLine();
      std::cout << "\n";
    }
    
  }
  return(0);
}
