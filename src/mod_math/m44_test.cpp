/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2009 - Jan Reucker (original author)
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

#include "matrix44.h"

/**
 * \file m44_test.cpp
 *
 * Unit test for the 4x4 matrix template class.
 * 
 */
int main()
{
  int pass = 0;
  int fail = 0;
  
  // A float 4x4 matrix initialized to all zeroes
  CRRCMath::Matrix44f  mf(0.0f);
  
  // A float 4x4 identity matrix
  CRRCMath::Matrix44f  id(true);
  
  std::cout << "Testing CRRCMath::Matrix44f::print()..." << std::endl;
  id.print();
  std::cout << "Testing CRRCMath::Matrix44f::printLine()..." << std::endl;
  id.printLine();
  std::cout << std::endl;

  {  
    // transposing, test 1: identity^T = identity
    CRRCMath::Matrix44f  tmp;
    
    tmp = id.trans();
    if (tmp.isEqualTo(id))
    {
      pass++;
    }
    else
    {
      fail++;
      std::cout << "transpose test 1 failed.\ntmp =\n";
      tmp.print();
      std::cout << "tmp should be\n";
      id.print();
    }
  }

  // transposing, test 2
  {
    CRRCMath::Matrix44f tmp ( 1, 5, 9, 13,
                              2, 6, 10, 14,
                              3, 7, 11, 15,
                              4, 8, 12, 16);
    CRRCMath::Matrix44f tmp2( 1, 2, 3, 4,
                              5, 6, 7, 8,
                              9, 10, 11, 12,
                              13, 14, 15, 16);
  
    if (tmp.isEqualTo(tmp2.trans()))
    {
      pass++;
    }
    else
    {
      fail++;
      std::cout << "transpose test 2 failed.\ntmp =\n";
      tmp.print();
      std::cout << "tmp should be\n";
      tmp2.trans().print();
    }
  }
  
  // inverting, test 1: identity.inv() == identity
  {
    CRRCMath::Matrix44f tmp = id.inv();
    
    if (id.isEqualTo(tmp))
    {
      pass++;
    }
    else
    {
      fail++;
      std::cout << "invert test 1 failed.\ntmp =\n";
      tmp.print();
      std::cout << "tmp should be\n";
      id.print();
    }
  }
  
  // inverting, test 2: translation matrix
  // inverting a pure translation matrix should result
  // in another pure translation matrix, with all
  // translation coefficients negated
  {
    // tmp performes a translation to (5|6|-3). This
    // matrix is created using the translation matrix ctor.
    CRRCMath::Matrix44f tmp(CRRCMath::Vector3(5.0f, 6.0f, -3.0f));
    
    // The expected value is constructed using the ordinary
    // identity matrix ctor. Therefore this test also validates
    // the two ctor implementations.
    CRRCMath::Matrix44f expected(true);
    expected.v[0][3] = -5.0f;
    expected.v[1][3] = -6.0f;
    expected.v[2][3] =  3.0f;

    tmp = tmp.inv();
    
    if (tmp.isEqualTo(expected))
    {
      pass++;
    }
    else
    {
      fail++;
      std::cout << "invert test 2 failed.\ntmp =\n";
      tmp.print();
      std::cout << "tmp should be\n";
      expected.print();
    }
  }
  
  // rotating, test 1
  {
    // create a matrix that rotates a point 90 deg around the z axis
    CRRCMath::Matrix44f tmp(CRRCMath::Vector3(0, 0, 1), 3.14159/2);
    // a vector in the x-y-plane
    CRRCMath::Vector3 v(1, 1, 0);
    // rotate the vector. expected result: (-1, 1, 0)
    CRRCMath::Vector3 result = tmp * v;
    float difference = CRRCMath::Vector3(result - CRRCMath::Vector3(-1, 1, 0)).length();
    if (difference < 0.0001)
    {
      pass++;
    }
    else
    {
      fail++;
      std::cout << "rotating, test 1 failed.\n";
      result.print("result = ", "\n");
      std::cout << "result should be (-1|1|0)\n";
      std::cout << "difference = " << difference << " (should be 0)" << std::endl;
    }
  }
  
  // rotating, test 2
  // rotate a point around an arbitrary axis:
  // - move the rotation axis to pass through the origin
  // - rotate around the axis
  // - move the result back
  {
    // the rotation axis is defined by the points (0|4|1) and (2|4|1)
    CRRCMath::Vector3 hinge1(0,4,1);
    CRRCMath::Vector3 hinge2(2,4,1);
    // so the axis points in this direction:
    CRRCMath::Vector3 axis = hinge2 - hinge1;

    // a matrix that moves hinge1 into the origin
    CRRCMath::Matrix44f move_orig(hinge2 * (-1));
    // a matrix that moves back
    CRRCMath::Matrix44f move_back(hinge2);
    // the rotation matrix. for simplicity, let's rotate 180 degrees
    CRRCMath::Matrix44f rotate(axis, 3.14159);
    
    // concatenate the matrices to create the final transformation matrix.
    // mult is not commutative, so always keep the matrices ordered from
    // right to left (first transformation step is the rightmost matrix):
    // transformed_point = move_back * rotate * move_orig * original_point;
    CRRCMath::Matrix44f xform = move_back * rotate * move_orig;
    
    // transform (1|4|0). expected: (1|4|2)
    CRRCMath::Vector3 result = xform * CRRCMath::Vector3(1, 4, 0);
    //~ result.print("result = ", "\n");
    
    float difference = CRRCMath::Vector3(result - CRRCMath::Vector3(1, 4, 2)).length();
    if (difference < 0.0001)
    {
      pass++;
    }
    else
    {
      fail++;
      std::cout << "rotating, test 2 failed.\n";
      result.print("result = ", "\n");
      std::cout << "result should be (1|4|2)\n";
      std::cout << "difference = " << difference << " (should be 0)" << std::endl;
    }
  }
  
  
  std::cout << std::endl << "Passed " << pass << "  Failed " << fail << std::endl;
  return 0;
}
