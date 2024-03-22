/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2006, 2008 Jens Wilhelm Wulf (original author)
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
  

#include "solve.h"

#include <iostream>
#include <math.h>
#include <stdlib.h>

// Init of static members:
flttype      SolveFourthOrder::precmin = 1.0;
unsigned int SolveFourthOrder::loopmax = 0;


const flttype valmin = 0.7;
const flttype valmax = 1.3;


int sign(flttype arg)
{
  // ob das funktioniert? Wegen des Vergleichs mit Null habe ich Zweifel...
  // immerhin sind das hier floats, keine ints...
  if (arg == 0) return(0);
  if (arg > 0) return(1);
  else return(-1);
}

inline flttype sqr(flttype val) { return(val*val); };
inline flttype pow3(flttype val) { return(val*val*val); };
inline flttype pow4(flttype val) { return(val*val*val*val); };

static int compmi(const void *m1, const void *m2)
{
  flttype a1 = *((flttype*)m1);
  flttype a2 = *((flttype*)m2);

  if (a1 < a2)
    return(-1);
  else if (a1 > a2)
    return(1);
  else
    return(0);
}


void SolveFourthOrder::solve()
{
  // Die Funktion ist
  //   n4*x^4+n3*x^3+n2*x^2+n1*x+n0=0
  // Die Ableitung ist
  //   4*n4*x^3 + 3*n3*x^2 + 2*n2*x + n1
  // Ich interessiere mich zunächst für die Extrempunkte der Funktion
  // (Nullstellen der Ableitung).
  //
  // Näheres dazu siehe Bronstein S. 131 und S.132 (alt).
  xi[0] = 0;
  xi[1] = 1;

  flttype r, s, t, p, q;

  r = 0.75*n3/n4;
  s = 0.50*n2/n4;
  t = 0.25*n1/n4;

  p = (3*s-r*r) / 3;
  q = (2*r*r*r/27) - (r*s/3) + t;

  flttype R, D, phi, y1;

  R = sign(q) * sqrt(fabs(p)/3);
  D = p*p*p/27+0.25*q*q;

  anz = 3;

  if (p > 0)
  {
    phi = asinh(q/(2*R*R*R));
    y1  = -2*R*sinh(phi/3);
  }
  else
  {
    if (D > 0)
    {
      phi = acosh(q/(2*R*R*R));
      y1  = -2*R*cosh(phi/3);
    }
    else
    {
      phi = acos(q/(2*R*R*R));
      y1  = -2*R*cos(phi/3);
      flttype y2  = -2*R*cos((phi/3)+(2*M_PI/3));
      flttype y3  = -2*R*cos((phi/3)+(4*M_PI/3));      
      xi[3] = y2 - (r/3);
      xi[4] = y3 - (r/3);
      anz = 5;      
    }
  }

  xi[2] = y1 - (r/3);
  
  qsort(xi, anz, sizeof(flttype), compmi);

  for (unsigned int cnt=0; cnt<anz; cnt++)
  {
    yi[cnt] = n4*pow4(xi[cnt])+n3*pow3(xi[cnt])+n2*sqr(xi[cnt])+n1*xi[cnt]+n0;
  }

  // Using a high initial precision increases average loopcnt, but decreases max loopcnt.
  // Use something in the range 1.0E-5 .. 1.0E-7
  flttype      prec       = 1.0E-5;
  bool         fSolved    = false;
  unsigned int cnt        = 1;
  unsigned int loopcnt    = 0;
  while (fSolved == false)
  {
    if (sign(yi[cnt-1]) != sign(yi[cnt]) &&
        (
         (xi[cnt-1] >= 0 && xi[cnt-1] <= 1) || (xi[cnt] >= 0 && xi[cnt] <= 1)
         ))
    {
      flttype xneu = 0;
      flttype yneu;
      flttype xa, xb, ya; //, yb;

      xa = xi[cnt-1];
      ya = yi[cnt-1];
      xb = xi[cnt];
      //yb = yi[cnt];

      // Bisektionsverfahren
      // Newton funktioniert hier nicht zuverlässig! Wenn man ihn an einer Stelle
      // mit kleiner Steigung startet, kann es sein, dass er hinter dem nächsten
      // Extrempunkt weitermacht, also gar nicht mehr im gewünschten Intervall.
      // Grenze für Abweichung evtl. kleiner machen, wenn Lösung nicht gefunden wird.
      while (fabs(xa-xb) > prec)
      {
        xneu = (xa+xb)/2;
        // Die folgende Zeile kann zu einem Fehler führen, siehe Email von Jan R. am 29.04.2006.
        // Eigentlich darf das nicht passieren, denn hier kann nur pow(x, y) einen Fehler auslösen,
        // und das auch nur, wenn x<0 und y keine ganze Zahl. Um diesem Fall vorzubeugen, wird 
        // die normale Multiplikation benutzt.
        // yneu = n4*pow(xneu, 4)+n3*pow(xneu, 3)+n2*sqr(xneu)+n1*xneu+n0;
        yneu = n4*pow4(xneu)+n3*pow3(xneu)+n2*sqr(xneu)+n1*xneu+n0;

        if (sign(yneu) == sign(ya))
        {
          xa = xneu;
          ya = yneu;
        }
        else
        {
          xb = xneu;
          //yb = yneu;
        }

        loopcnt++;
      }

      // Ergebnis überprüfen:
      if (xneu>=0 && xneu<=1)
      {
        flttype ell_a    = xneu * ell_p1;
        flttype ell_b    = xneu * ell_p2 + ell_p3;
        flttype ell_yoff = xneu * ell_p4 + ell_p5;
        flttype val      = sqr(ell_x/ell_a)+ sqr((ell_y-ell_yoff)/ell_b);

        if (val >= valmin && val <= valmax)
        {
          fSolved = true;
          res  = xneu;
        }
      }
    }

    //
    cnt++;
    if (cnt == anz && fSolved == false)
    {
      // Start again
      cnt = 1;
      // increase precision
      prec *= 0.1;
      if (prec < precmin)
      {
        precmin = prec;
        std::cout << "# precmin:\n";
        std::cout << precmin << "\n";
      }
      else if (prec == 0)
      {
        fSolved = true;
        res     = 5;
        std::cerr << "Thermal v3: no result found!\n";
      }
    }
  }
  
  if (loopcnt > loopmax)
    loopmax = loopcnt;
}

