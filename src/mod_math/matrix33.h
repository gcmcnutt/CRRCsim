/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2008 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2008 - Jan Reucker
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
#ifndef MATRIX33_H
# define MATRIX33_H

# include "vector3.h"

namespace CRRCMath
{
  /**
   * A 3x3 matrix of double values.
   *
   * @author Jens Wilhelm Wulf
   */
  class Matrix33 /*{{{*/
  {
    public:
     double v[3][3];

    public:

     Matrix33();

     Matrix33(double i00, double i01, double i02,
              double i10, double i11, double i12,
              double i20, double i21, double i22);

     Matrix33(const Matrix33& mb);

     /**
      * returns determinant of matrix
      */
     double det() const;

     /**
      * returns inverse matrix: does not check whether this is
      * possible (det() != 0)!
      */
     Matrix33 inv() const;

     /**
      * Operator: Zuweisung
      */
     Matrix33& operator=(const Matrix33& b);

     /**
      * Operator: Multiplikation mit Vector3
      */
     Vector3 operator*(const Vector3& b) const;

     /**
      * Operation: Erst die transponierte bilden, dann Multiplikation mit Vector3.
      *            Ist wahrscheinlich schneller als <tt>matrix.trans() * vector3</tt>.
      */
     Vector3 multrans(const Vector3& b) const;

     /**
      * Operator: Multiplikation mit Matrix33
      */
     Matrix33 operator*(const Matrix33& b) const;

     /**
      * Operator: Subtraktion
      */
     Matrix33 operator-(const Matrix33& b) const;

     /**
      * transponierte Matrix
      */
     Matrix33 trans() const;

     /**
      *
      */
     void print();

     /**
      *
      */
     void printLine();

  };
/*}}}*/
}
#endif
