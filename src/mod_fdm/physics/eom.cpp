// -*- mode: c; mode: fold -*-
/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2008 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2008 - Jan Reucker
 *   Copyright (C) 2008 - Olivier Bordes
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
//
#include "eom.h"

#include <iostream>
#include <math.h>
#include <cstdlib>

EOM_6DOF::EOM_6DOF()
{
}

EOM_6DOF::EOM_6DOF(CRRCMath::Vector3  initPosEarth,
                   CRRCMath::Vector3  initAng,
                   CRRCMath::Vector3  initVelBody,
                   double             initMass,
                   CRRCMath::Matrix33 initInertia)   
{
  inertia     = initInertia;
  dMass_inv   = 1.0/initMass;

  dGravity = 9.81; // m/s^2

  if (inertia.det() == 0)
  {
    std::cout << "unable to calculate inertia_inv\n";
    exit(-1);
  }
  inertia_inv = inertia.inv();

  // Umrechnung koerperfest->erdfest festlegen
  conv.init(initAng);

  // init velocity
  vel.init(initVelBody, CRRCMath::Vector3());

  // init position
  pos.init(initPosEarth, conv.local(initVelBody));

  // init angular velocity
  angvel.init(CRRCMath::Vector3(0,0,0), CRRCMath::Vector3(0,0,0));

  initAng.print("euler=", ", ");
  initVelBody.print("v_body=", ", ");
}


void EOM_6DOF::setGravity(double iGravity) 
{
  dGravity = iGravity;
}


void EOM_6DOF::print(std::string name)     
{
  std::cout << name;

  conv.updateEuler();

  pos.val.print("posEarth=", ", ");
  vel.val.print("velBody=", ", ");
  angvel.val.print("angvel=", ", ");
  conv.euler.print("euler=", "\n");
}


void EOM_6DOF::step(double             dT,
                    CRRCMath::Vector3  FBody,
                    CRRCMath::Vector3  MBody)        
{
  CRRCMath::Vector3 accel_body;
  CRRCMath::Vector3 angaccel_body;

  
  
  
  // Geschwindigkeit ins erdfeste Koordinatensystem transformieren:
  CRRCMath::Vector3 velEarth = conv.local(vel.val);
  
  // integrate velocity to position
  pos.step(dT, velEarth);

  // integrate angular velocity
  conv.step(dT, angvel.val);

  
  
  
  
  
  
  // Beschleunigungen addieren:
  //  1. Erdbeschleunigung, umgerechnet ins körperfeste Koordinatensystem
  //  2. Beschleunigung aus FBody
  //  3. der Rotationsterm
  accel_body = conv.body(CRRCMath::Vector3(0, 0, dGravity)) + (FBody*dMass_inv) + (vel.val*angvel.val);

  // integrate acceleration to velocity
  vel.step(dT, accel_body);

  // Drehbeschleunigungen addieren:
  angaccel_body = inertia_inv*( MBody - (angvel.val*(inertia*angvel.val)) );

  // integrate angular acceleration
  angvel.step(dT, angaccel_body);

  
  
  
  /*  
  vel.val.print("vel_body=", ", ");
  pos.val.print("pos=", ", ");
  velEarth.print("v_earth=", "\n");
*/
}

