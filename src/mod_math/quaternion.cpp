// -*- mode: c; mode: fold -*-
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
//
#include "quaternion.h"

#include <math.h>
#include <iostream>

#ifndef M_PI
# define M_PI 3.14159265359
#endif







/*******************************************************************************************/

CRRCMath::Vector3 CRRCMath::Quaternion::body(CRRCMath::Vector3 local)
{
  return(mat*local);
}

CRRCMath::Vector3 CRRCMath::Quaternion::local(Vector3 body)
{
  return(mat.multrans(body));
}

/*******************************************************************************************/

void CRRCMath::Quaternion_001::step(double            dT,
                                    CRRCMath::Vector3 omega)
{
  double ep0;
  double ep1;
  double ep2;
  double ep3;

  double l = length();

  if (l != 0)
  {
    double inv_eps = 1.0/length();

    ep0 = -omega.r[0]*e1.val - omega.r[1]*e2.val - omega.r[2]*e3.val;
    ep1 =  omega.r[0]*e0.val - omega.r[1]*e3.val + omega.r[2]*e2.val;
    ep2 =  omega.r[0]*e3.val + omega.r[1]*e0.val - omega.r[2]*e1.val;
    ep3 = -omega.r[0]*e2.val + omega.r[1]*e1.val + omega.r[2]*e0.val;

    e0.step(dT, 0.5*ep0*inv_eps);
    e1.step(dT, 0.5*ep1*inv_eps);
    e2.step(dT, 0.5*ep2*inv_eps);
    e3.step(dT, 0.5*ep3*inv_eps);
  }
  else
  {
    e0.step(dT, 0);
    e1.step(dT, 0);
    e2.step(dT, 0);
    e3.step(dT, 0);
  }

  /*
  l = length();

  if (l != 0)
  {
    double inv_eps = 1.0/length();
    e0.val *= inv_eps;
    e1.val *= inv_eps;
    e2.val *= inv_eps;
    e3.val *= inv_eps;
  }
   */
//  std::cout << "length= " << length() << "\n";
  
  update_mat();
}

void CRRCMath::Quaternion_001::update_mat()
{
  mat.v[0][0] = e0.val*e0.val + e1.val*e1.val - e2.val*e2.val - e3.val*e3.val;
  mat.v[0][1] = 2*(e1.val*e2.val + e0.val*e3.val);
  mat.v[0][2] = 2*(e1.val*e3.val - e0.val*e2.val);

  mat.v[1][0] = 2*(e1.val*e2.val - e0.val*e3.val);
  mat.v[1][1] = e0.val*e0.val - e1.val*e1.val + e2.val*e2.val - e3.val*e3.val;
  mat.v[1][2] = 2*(e2.val*e3.val + e0.val*e1.val);

  mat.v[2][0] = 2*(e1.val*e3.val + e0.val*e2.val);
  mat.v[2][1] = 2*(e2.val*e3.val - e0.val*e1.val);
  mat.v[2][2] = (e0.val*e0.val - e1.val*e1.val - e2.val*e2.val + e3.val*e3.val);
}

void CRRCMath::Quaternion_001::updateEuler()
{
  if (mat.v[0][2] >= 1)
    euler.r[1] = asin(1.0);
  else if (mat.v[0][2] <= -1)
    euler.r[1] = asin(-1.0);
  else
    euler.r[1] = asin(mat.v[0][2]);

  double b = cos(euler.r[1]);

  double c;
  double sign;

  c = mat.v[2][2] / b;
  if (c >= 1)
    c = acos(1.0);
  else if (c <= -1)
    c = acos(-1.0);
  else
    c = acos(c);

  if (mat.v[1][2] > 0)
    sign = 1;
  else
    sign = -1;

  euler.r[0] = c * sign;

  c = mat.v[0][0] / b;
  if (c >= 1)
    c = acos(1.0);
  else if (c <= -1)
    c = acos(-1.0);
  else
    c = acos(c);

  if (mat.v[0][1] > 0)
    sign = 1;
  else
    sign = -1;

  euler.r[2] = c * sign;
}

void CRRCMath::Quaternion_001::init(CRRCMath::Vector3 eulerAngle)
{
  double sphi   = sin(0.5*eulerAngle.r[0]);
  double cphi   = cos(0.5*eulerAngle.r[0]);
  double stheta = sin(0.5*eulerAngle.r[1]);
  double ctheta = cos(0.5*eulerAngle.r[1]);
  double spsi   = sin(0.5*eulerAngle.r[2]);
  double cpsi   = cos(0.5*eulerAngle.r[2]);

  e0.init(+cpsi*ctheta*cphi +spsi*stheta*sphi, 0);
  e1.init(+cpsi*ctheta*sphi -spsi*stheta*cphi, 0);
  e2.init(+cpsi*stheta*cphi +spsi*ctheta*sphi, 0);
  e3.init(-cpsi*stheta*sphi +spsi*ctheta*cphi, 0);
    
  update_mat();

//  std::cout << "Quaternion::init  mat=\n";
//  mat.print();

  updateEuler();

//  eulerAngle.print("Quaternion::init Euler=", "\n");
}

double CRRCMath::Quaternion_001::length()
{
  return(sqrt(e0.val*e0.val + e1.val*e1.val + e2.val*e2.val + e3.val*e3.val));
}

/*******************************************************************************************/

void CRRCMath::Quaternion_002::step(double            dT,
                                    CRRCMath::Vector3 omega)
{
//  std::cout << "length_A= " << length() << "\n";

  // Gleichung (2.13) aus [1]:
  double ep0 = 0.5 * (                   +omega.r[2]*e1.val -omega.r[1]*e2.val +omega.r[0]*e3.val);
  double ep1 = 0.5 * (-omega.r[2]*e0.val                    +omega.r[0]*e2.val +omega.r[1]*e3.val);
  double ep2 = 0.5 * (+omega.r[1]*e0.val -omega.r[0]*e1.val                    +omega.r[2]*e3.val);
  double ep3 = 0.5 * (-omega.r[0]*e0.val -omega.r[1]*e1.val -omega.r[2]*e2.val                   );

  e0.step(dT, ep0);
  e1.step(dT, ep1);
  e2.step(dT, ep2);
  e3.step(dT, ep3);

  // Länge wird erzwungen:
  double inv_eps = 1/length();
  
//  std::cout << "length_B= " << length() << "\n";
  
  e0.val *= inv_eps;
  e1.val *= inv_eps;
  e2.val *= inv_eps;
  e3.val *= inv_eps;
  
  update_mat();
}

void CRRCMath::Quaternion_002::update_mat()
{
  // Matrix nach Gleichung (2.8) aus [1]:
  mat.v[0][0] =  e0.val*e0.val - e1.val*e1.val - e2.val*e2.val + e3.val*e3.val;
  mat.v[0][1] =  2*(e0.val*e1.val + e2.val*e3.val);
  mat.v[0][2] =  2*(e0.val*e2.val - e1.val*e3.val);

  mat.v[1][0] =  2*(e0.val*e1.val - e2.val*e3.val);
  mat.v[1][1] = -e0.val*e0.val + e1.val*e1.val - e2.val*e2.val + e3.val*e3.val;
  mat.v[1][2] =  2*(e1.val*e2.val + e0.val*e3.val);

  mat.v[2][0] =  2*(e0.val*e2.val + e1.val*e3.val);
  mat.v[2][1] =  2*(e1.val*e2.val - e0.val*e3.val);
  mat.v[2][2] = -e0.val*e0.val - e1.val*e1.val + e2.val*e2.val + e3.val*e3.val;  
}

void CRRCMath::Quaternion_002::updateEuler()
{
  // Winkel nach Gleichungen (2.14) aus [1]:
  euler.r[0] = atan2(mat.v[1][2], mat.v[2][2]); // phi
  euler.r[1] = asin(-1*mat.v[0][2]);            // theta
  euler.r[2] = atan2(mat.v[0][1], mat.v[0][0]); // psi

  // Erste Tests zeigen, dass es mit den obigen Gleichungen nicht getan ist. Ich brauche:
  if (mat.v[2][2] == 0)
    euler.r[0] = 0;
  
  if (mat.v[0][0] == 0)
    euler.r[2] = 0;
}

void CRRCMath::Quaternion_002::init(CRRCMath::Vector3 eulerAngle)
{
  // Aus [1], ab (2.9)

  Matrix33 A = initFromEuler(eulerAngle);

  e3.init(0.5*sqrt(1 + A.v[0][0] + A.v[1][1] + A.v[2][2] ), 0);

  if (e3.val != 0)
  {
    double d = 1.0/(4*e3.val);

    e0.init(d*(A.v[1][2] - A.v[2][1]), 0);
    e1.init(d*(A.v[2][0] - A.v[0][2]), 0);
    e2.init(d*(A.v[0][1] - A.v[1][0]), 0);
  }
  else
  {
    e0.init(0.5*sqrt(1 + A.v[0][0] - A.v[1][1] - A.v[2][2]), 0);
    e1.init(0.5*sqrt(1 - A.v[2][2] - 2 * e0.val*e0.val), 0);
    e2.init(0.5*sqrt(1 - A.v[1][1] - 2 * e0.val*e0.val), 0);
  }

  update_mat();

  updateEuler();
}

double CRRCMath::Quaternion_002::length()
{
  return(sqrt(e0.val*e0.val + e1.val*e1.val + e2.val*e2.val + e3.val*e3.val));
}

CRRCMath::Matrix33 CRRCMath::Quaternion_002::initFromEuler(CRRCMath::Vector3 eul)
{
  Matrix33 mat2;

  double s0 = sin(eul.r[0]);
  double c0 = cos(eul.r[0]);
  double s1 = sin(eul.r[1]);
  double c1 = cos(eul.r[1]);
  double s2 = sin(eul.r[2]);
  double c2 = cos(eul.r[2]);

  // Berechnung der Matrix A aus [1], S.12, Formel (2.1)
  // Damit gilt mat2 * v_local = v_body
  mat2 = Matrix33(c1*c2,          s2*c1,          -s1,
                  c2*s1*s0-s2*c0, s2*s1*s0+c2*c0, c1*s0,
                  c2*s1*c0+s2*s0, s2*s1*c0-c2*s0, c1*c0);
  
  return(mat2);
}

void CRRCMath::Quaternion_002::convTest1()
{
  Matrix33 mat2 = initFromEuler(euler);

  // Vergleich mit der internen Matrix
  std::cout << "convTest1:\n";
  (mat2-mat).print();
}

/*******************************************************************************************/

void CRRCMath::Quaternion_003::step(double            dT,
                                    CRRCMath::Vector3 omega)
{
  double e_0 = e0.val;
  double e_1 = e1.val;
  double e_2 = e2.val;
  double e_3 = e3.val;
  
  /* Transform to quaternion rates (see Appendix E in [2]) */  
  double e_dot_0 = 0.5*( -omega.r[0]*e_1 - omega.r[1]*e_2 - omega.r[2]*e_3 );
  double e_dot_1 = 0.5*(  omega.r[0]*e_0 - omega.r[1]*e_3 + omega.r[2]*e_2 );
  double e_dot_2 = 0.5*(  omega.r[0]*e_3 + omega.r[1]*e_0 - omega.r[2]*e_1 );
  double e_dot_3 = 0.5*( -omega.r[0]*e_2 + omega.r[1]*e_1 + omega.r[2]*e_0 );
  
  /* Integrate using trapezoidal as before */
  e0.step(dT, e_dot_0);
  e1.step(dT, e_dot_1);
  e2.step(dT, e_dot_2);
  e3.step(dT, e_dot_3);

  /* calculate orthagonality correction  - scale quaternion to unity length */
  double inv_eps = 1/length();
  
//  std::cout << "length= " << length() << "\n";
  
  e0.val *= inv_eps;
  e1.val *= inv_eps;
  e2.val *= inv_eps;
  e3.val *= inv_eps;
  
  update_mat();
}

void CRRCMath::Quaternion_003::update_mat()
{
  double e_0 = e0.val;
  double e_1 = e1.val;
  double e_2 = e2.val;
  double e_3 = e3.val;
  
  mat.v[0][0] = e_0*e_0 + e_1*e_1 - e_2*e_2 - e_3*e_3;
  mat.v[0][1] = 2*(e_1*e_2 + e_0*e_3);
  mat.v[0][2] = 2*(e_1*e_3 - e_0*e_2);
  mat.v[1][0] = 2*(e_1*e_2 - e_0*e_3);
  mat.v[1][1] = e_0*e_0 - e_1*e_1 + e_2*e_2 - e_3*e_3;
  mat.v[1][2] = 2*(e_2*e_3 + e_0*e_1);
  mat.v[2][0] = 2*(e_1*e_3 + e_0*e_2);
  mat.v[2][1] = 2*(e_2*e_3 - e_0*e_1);
  mat.v[2][2] = e_0*e_0 - e_1*e_1 - e_2*e_2 + e_3*e_3;
}

void CRRCMath::Quaternion_003::updateEuler()
{
  double Phi, Theta, Psi;

  Theta = asin( -1*mat.v[0][2] );

  if( mat.v[0][0] == 0 )
    Psi = 0;
  else
    Psi = atan2( mat.v[0][1], mat.v[0][0] );

  if( mat.v[2][2] == 0 )
    Phi = 0;
  else
    Phi = atan2( mat.v[1][2], mat.v[2][2] );

  /* Resolve Psi to 0 - 359.9999 */
  if (Psi < 0 )      Psi += 2*M_PI;
  if (Psi >= 2*M_PI) Psi -= 2*M_PI;
  
  /* Resolve Phi to 0 - 359.9999 */
  if (Phi < 0 )      Phi += 2*M_PI;
  if (Phi >= 2*M_PI) Phi -= 2*M_PI;  
    
  euler.r[0] = Phi;
  euler.r[1] = Theta;
  euler.r[2] = Psi;
}

void CRRCMath::Quaternion_003::init(CRRCMath::Vector3 eulerAngle)
{
  double sphi   = sin(0.5*eulerAngle.r[0]);
  double cphi   = cos(0.5*eulerAngle.r[0]);
  double stheta = sin(0.5*eulerAngle.r[1]);
  double ctheta = cos(0.5*eulerAngle.r[1]);
  double spsi   = sin(0.5*eulerAngle.r[2]);
  double cpsi   = cos(0.5*eulerAngle.r[2]);

  e0.init(+cpsi*ctheta*cphi +spsi*stheta*sphi, 0);
  e1.init(+cpsi*ctheta*sphi -spsi*stheta*cphi, 0);
  e2.init(+cpsi*stheta*cphi +spsi*ctheta*sphi, 0);
  e3.init(-cpsi*stheta*sphi +spsi*ctheta*cphi, 0);
  
  update_mat();

//  std::cout << "Quaternion::init  mat=\n";
//  mat.print();

  updateEuler();

//  eulerAngle.print("Quaternion::init Euler=", "\n");
}

double CRRCMath::Quaternion_003::length()
{
  return(sqrt(e0.val*e0.val + e1.val*e1.val + e2.val*e2.val + e3.val*e3.val));
}
