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
#ifndef EOM_6DOF_H
#define EOM_6DOF_H

#include "../../mod_math/vector3.h"
#include "../../mod_math/matrix33.h"
#include "../../mod_math/quaternion.h"
#include "../../mod_math/intgr.h"


// jwtodo: Welches Integrationsverfahren ist zu benutzen?

/**
 * Equations of motion with six degrees of freedom.
 * 
 * Coordinate system: x -> north, y -> east, z -> up.
 * A flat world is modelled (no ball or ellipsoid, no 
 * geodetic or geocentric coordinates), as this is sufficient 
 * for a model flying sim.
 * 
 * @author Jens Wilhelm Wulf
 */
class EOM_6DOF
{
  public:
      
   /**
    * @param initPosEarth Position in erdfesten Koordinaten
    * @param initAng      Ausrichtung der Körperachsen in Euler-Winkeln bezogen auf
    *                     das erdfeste Koordinatensystem
    * @param initVelBody  Ausgangsgeschwindigkeit im körperfesten Koordinatensystem
    */
   EOM_6DOF(CRRCMath::Vector3  initPosEarth,
            CRRCMath::Vector3  initAng,
            CRRCMath::Vector3  initVelBody,
            double             initMass,
            CRRCMath::Matrix33 initInertia);
   
   EOM_6DOF();

   /**
    * Set gravity. Default is -9.81 m/s^2, so you only have to use this function
    * if you want a different value or use non SI-units.
    */
   void setGravity(double iGravity);
      
   /**
    * Ausführung eines Simulationsschritts.
    * 
    * @param FBody    Kräfte im körperfesten Koordinatensystem
    * @param MBody    Momente auf die Körperachsen
    */
   void step(double            dT,
             CRRCMath::Vector3 FBody,
             CRRCMath::Vector3 MBody);

   void print(std::string name);

  public:
   
   /**
    * erdfest: x, y, z
    */
   CRRCMath::Integrationsverfahren<CRRCMath::Vector3> pos;

   /**
    * körperfest: u, v, w
    */
   CRRCMath::Integrationsverfahren<CRRCMath::Vector3> vel;

   /**
    * körperfest: p, q, r
    */
   CRRCMath::Integrationsverfahren<CRRCMath::Vector3> angvel;
   
   CRRCMath::Quaternion_002          conv;
   
  private:

   /**
    * 
    */
   double dGravity;
   
   /**
    * 
    */
   CRRCMath::Matrix33 inertia;
   
   /**
    * 
    */
   CRRCMath::Matrix33 inertia_inv;
   
   /**
    * 
    */
   double   dMass_inv;   

};

#endif
