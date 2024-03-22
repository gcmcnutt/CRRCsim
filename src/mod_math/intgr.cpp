/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2008 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2005 - Jan Reucker
 *   Copyright (C) 2005 - Lionel Cailler
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
#include "intgr.h"

namespace CRRCMath
{

  inline template class Integrationsverfahren<double>;
  inline template class Integrationsverfahren<Vector3>;
  inline template class IntegrationsverfahrenB<double>;
  inline template class IntegrationsverfahrenB<Vector3>;
  inline template class IntegrationsverfahrenC<double>;
  
  template<class C> void Integrationsverfahren<C>::init(C Startwert, C AblStartwert)
  {
     AblAlt = AblStartwert;
     val    = Startwert;
  }

  template<class C> void Integrationsverfahren<C>::step(double dT, C AblNeu)
  {
     val    = val + (AblNeu*3 - AblAlt)*0.5*dT;
     AblAlt = AblNeu;
  }

  template<class C> void IntegrationsverfahrenB<C>::init(C Startwert, C AblStartwert)
  {
     AblAlt = AblStartwert;
     val    = Startwert;
  }

  template<class C> void IntegrationsverfahrenB<C>::step(double dT, C AblNeu)
  {
     val    = val + (AblNeu + AblAlt)*0.5*dT;
     AblAlt = AblNeu;
  }

  template<class C> void IntegrationsverfahrenC<C>::init(C Startwert, C AblStartwert)
  {
     val    = Startwert;
  }

  template<class C> void IntegrationsverfahrenC<C>::step(double dT, C AblNeu)
  {
     val    = val + AblNeu*dT;
  }

};
