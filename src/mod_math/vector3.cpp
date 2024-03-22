/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2008 - Jens Wilhelm Wulf (original author)
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
#include "vector3.h"

#include <iostream>
#include <math.h>

namespace CRRCMath
{

  double Vector3::length() const
  {
    return(sqrt(
                r[0]*r[0] + r[1]*r[1] + r[2]*r[2]
                ));
  }

  Vector3 const& Vector3::normalize()
  {
    double len = length();
    if (len != 0)
    {
      *this *= (1/len);
    }
    return *this;
  }

  void Vector3::print(std::string pre, std::string post) const
  {
    std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
    //  std::cout.precision(2);
    std::cout << pre << "(";

    std::cout.width(7);
    std::cout << r[0] << "|";

    std::cout.width(7);
    std::cout << r[1] << "|";

    std::cout.width(7);
    std::cout << r[2] << ")" << post;
  }
};
