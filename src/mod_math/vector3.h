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
#ifndef VECTOR3_H
#define VECTOR3_H

#include <Eigen/Dense>
#include <iostream>
#include <string>

namespace CRRCMath
{
  /**
   * Vector3 is now a typedef to Eigen::Vector3d for SIMD vectorization.
   * This replaces the hand-rolled implementation with ~509 call sites.
   */
  using Vector3 = Eigen::Vector3d;

  /**
   * Helper functions that were member functions in the old class.
   * These provide compatibility for code that used the old API.
   */

  // Access individual elements (old API used .r[i])
  // Eigen uses operator() or operator[], so v(0) becomes v(0) or v[0]

  // Print function (was Vector3::print)
  inline void print(const Vector3& v, std::string pre, std::string post)
  {
    std::cout << pre << v(0) << " " << v(1) << " " << v(2) << post;
  }

} // namespace CRRCMath

#endif // VECTOR3_H
