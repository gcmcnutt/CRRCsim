/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2008-2009 - Jens Wilhelm Wulf (original author)
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
#ifndef QUATERNION_H
# define QUATERNION_H

# include "vector3.h"
# include "matrix33.h"
# include "intgr.h"

namespace CRRCMath
{
  
  /**
   * Transformation und Integration von Winkellagen
   * 
   * Ich gehe nach der Beschreibung aus [1], was die Bildung der 
   * Euler-Winkel angeht und die Definition der Koordinatensysteme etc. 
   * Dort heisst es unter anderem:
   *    "Suppose you have the plane in its position and want to determine 
   *     phi, theta, psi:
   *
   *       (1) roll angle psi is obtained by rotating inertial system about its z-axis
   *           until its y-axis becomes perpendicular to the plane of the inertial z-axis
   *           and the body-fixed x-axis
   *       (2) pitch angle theta is obtained by rotating the new inertial system
   *           (created by step 1) about its new y-axis until its x-axis overlaps with
   *           the body-fixed x-axis
   *       (3) yaw angle phi is finally obtained by continuing to rotate the new
   *           inertial system about its new x-axis until its y-axis lines up with the
   *           body fixed y-axis."
   *
   * Angenommen, die Lage ist phi=theta=psi=0, dann stimmen die Koordinatensysteme 
   * body und local überein. Eine positive Drehrate in p/q/r führt dann auch zu einem
   * positiven Winkel phi/theta/psi.
   *
   * [1]: "Six-Degree-of-Freedom Sensor Fish Design: Governing Equations and Motion Modeling".
   *
   * jwtodo: welches Integrationsverfahren? Wie anwenden?
   * 
   * @author Jens Wilhelm Wulf
   */
  class Quaternion
  {
    public:
     virtual  ~Quaternion() {}

     /**
      * converts local to body
      */
     CRRCMath::Vector3 body(CRRCMath::Vector3 local);

     /**
      * converts body to local
      */
     CRRCMath::Vector3 local(CRRCMath::Vector3 body);

     /**
      * phi, theta, psi
      */
     CRRCMath::Vector3 euler;

     /**
      * local to body
      */
     CRRCMath::Matrix33 mat;          
  };
  

  /**
   * Transformation und Integration von Winkellagen
   *
   * Mit beiden Integrationsverfahren schlecht
   * 
   *
   * @author Jens Wilhelm Wulf
   */
  class Quaternion_001 : public Quaternion 
  {
    public:
     void init(CRRCMath::Vector3 eulerAngle);

     /**
      *
      */
     void updateEuler();

     /**
      * @param omega Angular velocity (p, q, r)
      */
     void step(double            dT,
               CRRCMath::Vector3 omega);
     
    private:

     void   update_mat();
     double length();

     IntegrationsverfahrenB<double> e0, e1, e2, e3;
  };


  /**
   * Transformation und Integration von Winkellagen
   * 
   * Mit 'Integrationsverfahren' gibt es Fehler, mit 'IntegrationsverfahrenB' ist es OK.
   * 
   * [1]: "Six-Degree-of-Freedom Sensor Fish Design: Governing Equations and Motion Modeling".
   *      PNNl-14779.pdf
   *
   * @author Jens Wilhelm Wulf
   */
  class Quaternion_002 : public Quaternion 
  {
    public:
     void init(CRRCMath::Vector3 eulerAngle);

     /**
      *
      */
     void updateEuler();

     /**
      * @param omega Angular velocity (p, q, r)
      */
     void step(double            dT,
               CRRCMath::Vector3 omega);
     
     /**
      * debugging, test: calculates conversion matrix 'local to body' from euler angles,
      * compares to internal matrix
      */
     void convTest1();

    private:

     Matrix33 initFromEuler(CRRCMath::Vector3 eul);
     
     void   update_mat();
     double length();

     IntegrationsverfahrenB<double> e0, e1, e2, e3;
  };

  /**
   * Transformation und Integration von Winkellagen
   *
   * Abgeschaut von CRRCSim, scheint OK (zumindest mit IntegrationsverfahrenB).
   * 
   * @author Jens Wilhelm Wulf
   */
  class Quaternion_003 : public Quaternion 
  {
    public:
     void init(CRRCMath::Vector3 eulerAngle);

     /**
      *
      */
     void updateEuler();

     /**
      * @param omega Angular velocity (p, q, r)
      */
     void step(double            dT,
               CRRCMath::Vector3 omega);
     
    private:

     void   update_mat();
     double length();

     IntegrationsverfahrenB<double> e0, e1, e2, e3;
  };
  
}


#endif
