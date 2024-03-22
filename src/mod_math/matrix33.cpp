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
#include "matrix33.h"

#include <iostream>

namespace CRRCMath
{
  Matrix33::Matrix33()
  {
    for (unsigned char m=0; m<3; m++)
      for (unsigned char n=0; n<3; n++)
        v[m][n] = 0;
  }

  Matrix33::Matrix33(double i00, double i01, double i02,
                     double i10, double i11, double i12,
                     double i20, double i21, double i22)
  {
    v[0][0] = i00;
    v[0][1] = i01;
    v[0][2] = i02;
    v[1][0] = i10;
    v[1][1] = i11;
    v[1][2] = i12;
    v[2][0] = i20;
    v[2][1] = i21;
    v[2][2] = i22;
  }

  Matrix33::Matrix33(const Matrix33& mb)
  {
    for (unsigned char m=0; m<3; m++)
      for (unsigned char n=0; n<3; n++)
        v[m][n] = mb.v[m][n];
  }

  double Matrix33::det() const
  {
    return (
            v[0][0]*v[1][1]*v[2][2] + v[0][1]*v[1][2]*v[2][0]
            + v[0][2]*v[1][0]*v[2][1] - v[0][2]*v[1][1]*v[2][0]
            - v[0][1]*v[1][0]*v[2][2] - v[1][2]*v[2][1]*v[0][0]
            );
  }

  Matrix33 Matrix33::trans() const
  {
    return(Matrix33(v[0][0], v[1][0], v[2][0],
                    v[0][1], v[1][1], v[2][1],
                    v[0][2], v[1][2], v[2][2]));
  }

  Matrix33 Matrix33::inv() const
  {
    // Compute the inverse of a general matrix using Cramers rule.
    // I guess googling for cramers rule gives tons of references
    // for this. :)
    double rdet = 1.0/det();

    double i00 = rdet*(v[1][1]*v[2][2]-v[1][2]*v[2][1]);
    double i10 = rdet*(v[1][2]*v[2][0]-v[1][0]*v[2][2]);
    double i20 = rdet*(v[1][0]*v[2][1]-v[1][1]*v[2][0]);
    double i01 = rdet*(v[0][2]*v[2][1]-v[0][1]*v[2][2]);
    double i11 = rdet*(v[0][0]*v[2][2]-v[0][2]*v[2][0]);
    double i21 = rdet*(v[0][1]*v[2][0]-v[0][0]*v[2][1]);
    double i02 = rdet*(v[0][1]*v[1][2]-v[0][2]*v[1][1]);
    double i12 = rdet*(v[0][2]*v[1][0]-v[0][0]*v[1][2]);
    double i22 = rdet*(v[0][0]*v[1][1]-v[0][1]*v[1][0]);

    return Matrix33( i00, i01, i02,
                     i10, i11, i12,
                     i20, i21, i22 );
  }

  Matrix33& Matrix33::operator=(const Matrix33& b)
  {
    for (unsigned char m=0; m<3; m++)
      for (unsigned char n=0; n<3; n++)
        v[m][n] = b.v[m][n];

    return *this;
  }

  Vector3 Matrix33::operator*(const Vector3& b) const
  {
    Vector3 tmp;

    for (unsigned char m=0; m<3; m++)
      for (unsigned char n=0; n<3; n++)
        tmp.r[m] += v[m][n]*b.r[n];

    return tmp;
  }

  Vector3 Matrix33::multrans(const Vector3& b) const
  {
    Vector3 tmp;

    for (unsigned char m=0; m<3; m++)
      for (unsigned char n=0; n<3; n++)
        tmp.r[m] += v[n][m]*b.r[n];

    return tmp;
  }

  Matrix33 Matrix33::operator*(const Matrix33& b) const
  {
    Matrix33 tmp;

    for (unsigned char m=0; m<3; m++)
    {
      for (unsigned char n=0; n<3; n++)
      {
        tmp.v[m][n] = 0;
        for (unsigned char i=0; i<3; i++)
          tmp.v[m][n] += v[m][i]*b.v[i][n];
      }
    }

    return tmp;
  }

  Matrix33 Matrix33::operator-(const Matrix33& b) const
  {
    Matrix33 tmp;

    for (unsigned char m=0; m<3; m++)
    {
      for (unsigned char n=0; n<3; n++)
      {
        tmp.v[m][n] = v[m][n]-b.v[m][n];
      }
    }

    return tmp;
  }

  void Matrix33::print()
  {
    for (unsigned char m=0; m<3; m++)
    {
      for (unsigned char n=0; n<3; n++)
        std::cout << v[m][n] << ", ";
      std::cout << "\n";
    }
    std::cout << "\n";
  }
  
  void Matrix33::printLine()
  {
    for (unsigned char m=0; m<3; m++)
    {
      for (unsigned char n=0; n<3; n++)
        std::cout << v[m][n] << " ";
    }
  }
};
