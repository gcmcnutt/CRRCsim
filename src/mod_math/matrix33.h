/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2008 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2008 - Jan Reucker
 *   Copyright (C) 2026 - Eigen migration for SIMD vectorization
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 */
#ifndef MATRIX33_H
#define MATRIX33_H

#include <Eigen/Dense>
#include "vector3.h"

namespace CRRCMath
{
  /**
   * Matrix33 is now a typedef to Eigen::Matrix3d for SIMD vectorization.
   *
   * IMPORTANT: Eigen uses COLUMN-MAJOR storage by default, but the old code
   * used ROW-MAJOR with v[row][col] access. Using RowMajor here for compatibility.
   *
   * Old access: matrix.v[row][col]
   * New access: matrix(row, col)
   */
  using Matrix33 = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;

  /**
   * Factory function to create Matrix33 from 9 values (row-major order).
   * Replaces the old Matrix33(r00,r01,r02, r10,r11,r12, r20,r21,r22) constructor.
   */
  inline Matrix33 make_matrix33(
      double r00, double r01, double r02,
      double r10, double r11, double r12,
      double r20, double r21, double r22)
  {
    Matrix33 m;
    m << r00, r01, r02,
         r10, r11, r12,
         r20, r21, r22;
    return m;
  }

  /**
   * Helper functions that were member functions in the old Matrix33 class.
   */

  // Transpose then multiply by vector: matrix.transpose() * vec
  // More efficient than forming transpose explicitly
  inline Vector3 multrans(const Matrix33& m, const Vector3& v)
  {
    return m.transpose() * v;
  }

  // Print functions
  inline void print(const Matrix33& m)
  {
    for (int row = 0; row < 3; row++)
    {
      for (int col = 0; col < 3; col++)
        std::cout << m(row, col) << ", ";
      std::cout << "\n";
    }
    std::cout << "\n";
  }

  inline void printLine(const Matrix33& m)
  {
    for (int row = 0; row < 3; row++)
    {
      for (int col = 0; col < 3; col++)
        std::cout << m(row, col) << " ";
    }
  }

} // namespace CRRCMath

#endif // MATRIX33_H
