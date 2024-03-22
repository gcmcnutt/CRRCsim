/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2008 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2007, 2008 - Jan Reucker
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
#ifndef INTGR_H
# define INTGR_H

# include "vector3.h"

/** \brief this namespace contains pure math
 *
 * [1] Bronstein/Semendjajew, 'Taschenbuch der Mathematik', B. G. Teubner Verlagsgesellschaft.
 * 
 * [2] Friedrich Weller, 'Numerische Mathematik für Ingenieure und Naturwissenschaftler', Vieweg Verlag.
 * 
 * jwtodo: welches Integrationsverfahren? Wie anwenden?
 * 
 */
namespace CRRCMath
{

  /**
   * Adams-Bashforth algorithm
   * 
   * Einfachste Version des Adams-Bashforth (lineares explizites Zweischrittverfahren nach 
   * der Methode von Adams-Bashforth).
   * 
   * Herleitung nach [2].
   * 
   * \f[
       y(x_{i+1}) - y(x_i) = \int_{x_i}^{x_{i+1}} f(x, y(x)) \, dx
     \f]
   * Die Ableitung im Punkt \f$(x,y)\f$ ist \f$f(x,y)\f$.
   * 
   * \f[
       f_i = f(x_i, y_i)
     \f]
   * 
   * \f[
       f_{i-1} = f(x_{i-1}, y_{i-1})
     \f]
   * 
   * Die Ableitung wird linear interpoliert/extrapoliert:
   * \f[
       f(x) = f_i + \frac{f_i - f_{i-1}}{h} \, (x - x_i)
     \f]
   * 
   * mit \f$ h = x_{i+1} - x_i = x_i - x_{i-1}\f$. Der obige Ausdruck für \f$f(x)\f$ wird in die erste 
   * Gleichung eingesetzt und umgeformt. Man erhält dann
   * 
   * \f[
       y(x_{i+1}) = y(x_i) + \frac{h}{2} \, (3\, f_i - f_{i-1})
     \f]
   *
   * @author Jens Wilhelm Wulf
   */
  template<class C> class Integrationsverfahren /*{{{*/
  {
    public:
     
     /**
      * virtual base class should have a virtual dtor
      */
     virtual ~Integrationsverfahren() {};

      /**
      * Festlegung von Startwert und Ableitung zum Startzeitpunkt
      */
     virtual void init(C Startwert, C AblStartwert);

     /**
      * Ausführung eines Integrationsschritts.
      */
     virtual void step(double dT, C AblNeu);

     /**
      * der integrierte Wert
      */
     C val;

    private:
     C AblAlt;
  };

  /**
   *
   * Trapezmethode, Verfahren von Heun. Abschnitt 9.3.3 in [2]
   * 
   * jwtodo
   * 
   * Zumindest lassen die Kommentare im originalen CRRCSim darauf schließen. Wenn ich aber versuche, 
   * das nachzuvollziehen, komme ich (bisher) noch nicht zu dem Schluss, dass diese Implementierung 
   * dieses Verfahren korrekt abbildet. Es ist einfach etwas anderes.
   * 
   * @author Jens Wilhelm Wulf
   */
  template<class C> class IntegrationsverfahrenB /*{{{*/
  {
    public:
     
     /**
      * virtual base class should have a virtual dtor
      */
     virtual ~IntegrationsverfahrenB() {};

     /**
      * Festlegung von Startwert und Ableitung zum Startzeitpunkt
      */
     virtual void init(C Startwert, C AblStartwert);

     /**
      * Ausführung eines Integrationsschritts.
      */
     virtual void step(double dT, C AblNeu);

     /**
      * der integrierte Wert
      */
     C val;

    private:
     C AblAlt;
  };

  /**
   * Polygonzugverfahren von Euler
   * 
   * Abschnitt 7.1.2.9 in [1]
   * 
   * Abschnitt 9.3.2   in [2]
   * 
   * @author Jens Wilhelm Wulf
   */
  template<class C> class IntegrationsverfahrenC /*{{{*/
  {
    public:
     
     /**
      * virtual base class should have a virtual dtor
      */
     virtual ~IntegrationsverfahrenC() {};

     /**
      * Festlegung von Startwert und Ableitung zum Startzeitpunkt
      */
     virtual void init(C Startwert, C AblStartwert);

     /**
      * Ausführung eines Integrationsschritts.
      */
     virtual void step(double dT, C AblNeu);

     /**
      * der integrierte Wert
      */
     C val;
  };
  
}
#endif
