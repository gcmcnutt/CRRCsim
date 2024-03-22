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
#ifndef MATRIX44_H
#define MATRIX44_H

#include "vector3.h"

namespace CRRCMath
{
  /**
   * A template class for 4x4 matrices.
   *
   * @author Jan Reucker
   */
  template <class T>
  class Matrix44
  {
    private:
      /** typed abs
       *  abs() function for the type T
       */
      static inline T t_abs_(const T val)
      {
        return (val < (T)0) ? -val : val;
      }
      
    public:
     T v[4][4];

    public:
      /**
       * Create a 4x4 matrix
       *
       * This is the default constructor. It does not
       * initialize the matrix elements unless ident
       * is set to true. In this case, the matrix is
       * initialized as an ident matrix.
       *
       * \param ident Initialize as ident matrix?
       */
      Matrix44(bool ident = false)
      {
        if (ident)
        {
          makeIdent();
        }
      }
      
      /** 
       * Create a 4x4 matrix and initialize it with "iv"
       *
       * \param iv    initial value for all matrix elements
       */
      Matrix44(T iv)
      {
        for (unsigned char m = 0; m < 4; m++)
        {
          for (unsigned char n = 0; n < 4; n++)
          {
            v[m][n] = iv;
          }
        }
      }

      /** 
       * Create a 4x4 matrix and initialize it with
       * the given values.
       */
      Matrix44( T i00, T i01, T i02, T i03,
                T i10, T i11, T i12, T i13,
                T i20, T i21, T i22, T i23,
                T i30, T i31, T i32, T i33)
      {
        v[0][0] = i00;
        v[0][1] = i01;
        v[0][2] = i02;
        v[0][3] = i03;
        v[1][0] = i10;
        v[1][1] = i11;
        v[1][2] = i12;
        v[1][3] = i13;
        v[2][0] = i20;
        v[2][1] = i21;
        v[2][2] = i22;
        v[2][3] = i23;
        v[3][0] = i30;
        v[3][1] = i31;
        v[3][2] = i32;
        v[3][3] = i33;
      }

      /**
       * Create a translation matrix
       *
       * Creates a 4x4 matrix which represents a
       * translation in 3D space.
       *
       * \param translation  Translation vector
       */
      Matrix44(Vector3 const& translation)
      {
        makeTranslation(translation);
      }
      
      /**
       * Turn matrix into a translation matrix
       *
       * Initialize all elements to represent the given
       * translation in 3D space.
       *
       * \param translation   translation vector
       */
      void makeTranslation(Vector3 const& translation)
      {
        makeIdent();
        v[0][3] = translation.r[0];
        v[1][3] = translation.r[1];
        v[2][3] = translation.r[2];
      }
      
      /**
       * Create a rotation matrix
       *
       * Creates a 4x4 matrix which represents a
       * rotation in 3D space.
       *
       * \param axis  rotation axis
       * \param angle rotation angle in radians
       */
      Matrix44(Vector3 const& axis, T angle)
      {
        makeRotation(axis, angle);
      }
      
      /**
       * turn this matrix into an identity matrix
       */
      void makeIdent()
      {
        for (unsigned char m = 0; m < 4; m++)
        {
          for (unsigned char n = 0; n < 4; n++)
          {
            v[m][n] = 0;
          }
          v[m][m] = 1;
        }
      }
      
      /**
       * Turn matrix into a rotation matrix
       *
       * Initialize all elements to represent the given
       * rotation in 3D space.
       *
       * Reference: http://www.fastgraph.com/makegames/3drotation/
       *
       * \param axis  rotation axis
       * \param angle rotation angle in radians
       */
      void makeRotation(Vector3 const& axis, T angle)
      {
        T c = (T)cos(angle);
        T s = (T)sin(angle);
        T t = (T)(1 - c);
        
        Vector3 normaxis(axis);
        normaxis.normalize();
        T x = (T)normaxis.r[0];
        T y = (T)normaxis.r[1];
        T z = (T)normaxis.r[2];
        
        v[0][0] = t * x * x + c;
        v[0][1] = t * x * y - s * z;
        v[0][2] = t * x * z + s * y;
        v[0][3] = 0;
        
        v[1][0] = t * x * y + s * z;
        v[1][1] = t * y * y + c;
        v[1][2] = t * y * z + s * x;
        v[1][3] = 0;
        
        v[2][0] = t * x * z - s * y;
        v[2][1] = t * y * z + s * x;
        v[2][2] = t * z * z + c;
        v[2][3] = 0;
        
        v[3][0] = 0;
        v[3][1] = 0;
        v[3][2] = 0;
        v[3][3] = 1;
      }

      /**
       * returns determinant of matrix
       * \todo This is still the 3x3 algorithm!
       */
      T det() const
      {
        return (
                v[0][0]*v[1][1]*v[2][2] + v[0][1]*v[1][2]*v[2][0]
                + v[0][2]*v[1][0]*v[2][1] - v[0][2]*v[1][1]*v[2][0]
                - v[0][1]*v[1][0]*v[2][2] - v[1][2]*v[2][1]*v[0][0]
                );
      }

      /**
       * returns inverse matrix. Currently does not check whether
       * this is possible (det() != 0)!
       */
      Matrix44 inv() const
      {
        Matrix44 tmp;
        Matrix44 ret(true);   // set to ident matrix

        tmp = *this;

        for (int i = 0 ; i < 4 ; i++)
        {
          T val = tmp.v[i][i];
          int ind = i;
          int j;

          for (j = i + 1; j < 4 ; j++)
          {
            if ( t_abs_ ( tmp.v[i][j] ) > t_abs_(val) )
            {
              ind = j;
              val = tmp.v[i][j] ;
            }
          }

          if ( ind != i )
          {                   /* swap columns */
            for ( j = 0 ; j < 4 ; j++ )
            {
              T t ;
              t = ret.v[j][i]; 
              ret.v[j][i] = ret.v[j][ind];
              ret.v[j][ind] = t;
              t = tmp.v[j][i];
              tmp.v[j][i] = tmp.v[j][ind];
              tmp.v[j][ind] = t;
            }
          }

          // if ( val == SG_ZERO)
          //~ if ( t_abs_(val) <= FLT_EPSILON )
          //~ {
            //~ ulSetError ( UL_WARNING, "sg: ERROR - Singular matrix, no inverse!" ) ;
            //~ sgMakeIdentMat4 ( dst ) ;  /* Do *something* */
            //~ return;
          //~ }

          T ival = (T)1 / val ;

          for (j = 0 ; j < 4 ; j++)
          {
            tmp.v[j][i] *= ival ;
            ret.v[j][i] *= ival ;
          }

          for (j = 0; j < 4; j++)
          {
            if ( j == i )
              continue ;

            val = tmp.v[i][j] ;

            for ( int k = 0 ; k < 4 ; k++ )
            {
              tmp.v[k][j] -= tmp.v[k][i] * val ;
              ret.v[k][j] -= ret.v[k][i] * val ;
            }
          }
        }
        return ret;
      }

      /**
       * Operator: Assignment
       */
      Matrix44& operator=(const Matrix44& rhs)
      {
        /// \todo check for assignment to this first
        for (unsigned char m = 0; m < 4; m++)
          for (unsigned char n = 0; n < 4; n++)
            v[m][n] = rhs.v[m][n];

        return *this;
      }

      /**
       * Operator: Multiplication with Vector3
       * This can be used to transform a Vector3 using
       * the given matrix as a transformation matrix.
       *
       * The Vector3 is treated like an 1x4 vector with the
       * fourth row (the homogenous w coordinate) set to 1.
       */
      Vector3 operator*(const Vector3& rhs) const
      {
        return Vector3(v[0][0] * rhs.r[0] + v[0][1] * rhs.r[1] + v[0][2] * rhs.r[2] + v[0][3],
                       v[1][0] * rhs.r[0] + v[1][1] * rhs.r[1] + v[1][2] * rhs.r[2] + v[1][3],
                       v[2][0] * rhs.r[0] + v[2][1] * rhs.r[1] + v[2][2] * rhs.r[2] + v[2][3]);
      }

      /**
      * Operation: Erst die transponierte bilden, dann Multiplikation mit Vector3.
       *            Ist wahrscheinlich schneller als <tt>matrix.trans() * vector3</tt>.
       */
      //~ Vector3 multrans(const Vector3& b) const;

      /**
       * Operator: Multiplication with a Matrix44
       *
       * Performs the operation: ret = this * rhs
       */
      Matrix44 operator*(const Matrix44& rhs) const
      {
        Matrix44 tmp;

        for (unsigned char m = 0; m < 4; m++)
        {
          for (unsigned char n = 0; n < 4; n++)
          {
            tmp.v[m][n] = 0;
            for (unsigned char i = 0; i < 4; i++)
              tmp.v[m][n] += v[m][i] * rhs.v[i][n];
          }
        }

        return tmp;
      }
      

     /**
      * Operator: Subtraction
      *
      * Performs the operation: ret = this - rhs
      */
      Matrix44 operator-(const Matrix44& rhs) const
      {
        Matrix44 tmp;

        for (unsigned char m = 0; m < 4; m++)
        {
          for (unsigned char n = 0; n < 4; n++)
          {
            tmp.v[m][n] = v[m][n] - rhs.v[m][n];
          }
        }

        return tmp;
      }

     /**
      * return the transposed matrix
      */
      Matrix44 trans() const
      {
        return(Matrix44(v[0][0], v[1][0], v[2][0], v[3][0],
                        v[0][1], v[1][1], v[2][1], v[3][1],
                        v[0][2], v[1][2], v[2][2], v[3][2],
                        v[0][3], v[1][3], v[2][3], v[3][3]));
      }
      
      /**
       * Compare two matrices
       *
       * With floating point values, comparing for equality
       * using operator== may lead to problems due to numerical
       * inaccuracies. This method calculates the difference
       * between two matrices and then compares all elements
       * of the resulting matrix to the given delta. If all
       * elements are within -delta < v[][] < delta, the result
       * is true, else it's false.
       */
      bool isEqualTo(const Matrix44& rhs, T delta = 1e-5)
      {
        bool equal = true;
        Matrix44 tmp = *this - rhs;
        
        for (unsigned char m = 0; m < 4; m++)
        {
          for (unsigned char n = 0; n < 4; n++)
          {
            if ((tmp.v[m][n] < -delta) || (tmp.v[m][n] > delta))
            {
              equal = false;
            }
          }
        }
        
        return equal;
      }

      /**
       *
       */
      void print()
      {
        for (unsigned char m = 0; m < 4; m++)
        {
          for (unsigned char n = 0; n < 4; n++)
            std::cout << v[m][n] << ", ";
          std::cout << "\n";
        }
        std::cout << "\n";
      }
      
      /**
       *
       */
      void printLine()
      {
        for (unsigned char m = 0; m < 4; m++)
        {
          for (unsigned char n = 0; n < 4; n++)
            std::cout << v[m][n] << " ";
        }
      }
  };
  
  typedef Matrix44<float>   Matrix44f;
  typedef Matrix44<double>  Matrix44d;

} // namespace CRRCMath

#endif // MATRIX44_H
