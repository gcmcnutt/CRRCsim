// -*- mode: c; mode: fold -*-
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

#include "thermikschale.h"

#include <math.h>
#include <iostream>
#include "tschalen.h"
#include "solve.h"
#include "../../mod_misc/lib_conversions.h"

// deltat muss in jedem Fall positiv sein, damit man gescheite Längen
// der Senkrechten am oberen und unteren Ende bekommt (starke Krümmung).
const flttype deltat   = 0.005;

const flttype anzfact  = 0.002;

// anzfacts=1 zeigt Senkrechte so wie sie berechnet wird, 
// anzfacts=-1 zeigt Senkrechte so wie sie zur Berechnung der Oberfläche benutzt wird
const flttype anzfacts = 1;


inline flttype sqr(flttype val) { return(val*val); };



void ThermikSchale::init()
{
  A1 = (ou_x - ol_x);
  A2 = (ol_y - il_y);
  A3 = (ou_y - iu_y - ol_y +il_y);
  A4 = (iu_y -il_y);

  nenner = - ol_x * ou_y + ol_y * ou_x - il_y * ou_x + iu_y * ol_x;    
}

void ThermikSchale::getDir(flttype  x,   flttype y,
                           flttype& dx,  flttype& dy,
                           flttype& dxs, flttype& dys,
                           flttype& t) /*{{{*/
{
  flttype l, len;

  // todo: wenn dieser Code so bleibt sollte man die großen Konstanten vorberechnen.
  //
  // Im Bereich der Gerade?
  //
  // Wenn solver keine Lösung finden kann:
  //       In diesem Fall gibt es solver.res = 5,
  //       diese Funktion liefert dx=dy=dxs=dys=0 und t=5. Das ist zwar kein gescheites 
  //       Ergebnis, aber damit läuft CRRCSim immerhin weiter.

  if (A1 == 0)
  {
    t = x / ol_x;
  }
  else
  {
    flttype p = (A3 * x - A1 * y - A4 * ol_x + A1 * il_y) / nenner;
    flttype q = A4 * x / nenner;

    t = -p/2 + sqrt(sqr(p)/4 - q);
  }
  dx = 0;
  dy = 0;
  dxs = 0;
  dys = 0;

  if (t >= 0 && t <= 1)
  {
    if (A1 == 0)
      l = (y - il_y - t*A2)/(A3*t+A4);
    else
      l = (x - ol_x * t) / (t*A1);

    if (l>=0 && l<=1 && t > 2*deltat) /*{{{*/
    {
      // Es ist im Bereich der Gerade. Der Richtungsvektor ist
      dx  = l*t*A1;
      dy  = l*  (A4 + t*A3);
      len = sqrt(dx*dx+dy*dy);
      dx  = dx/len;
      dy  = dy/len;
      
      if ((x>0 && dy>0) || (x<0 && dy<0) )
      {
        dx *= -1;
        dy *= -1;
      }

      // senkrecht dazu ist (nicht exakt, aber genau genug):
      dxs = deltat*(ol_x + l*A1);
      dys = dxs * t*A1/((A4 + t*A3));
    }
    /*}}}*/

    else if (l<0)     /*{{{*/
    {
      // Es könnte im Bereich der unteren Ellipse liegen. Für t=1 gilt
      flttype a    = ol_x;
      flttype b    = ol_y;
      flttype yoff = ol_y;

      if (sqr(x/a)+ sqr((y-yoff)/b) < 1)
      {
        // Es liegt im Bereich der unteren Ellipse. Wie ist t?
        flttype c = A2;
        flttype d = dzl_y + c;
        flttype f = il_y - dzl_y;

        flttype n4=(sqr(c)-sqr(d))*sqr(ol_x);
        flttype n3=(2*c*il_y-2*d*f)*sqr(ol_x) - 2*c*sqr(ol_x)*y;
        flttype n2=(sqr(ol_x*y)-2*il_y*sqr(ol_x)*y+sqr(d*x)+(sqr(il_y)-sqr(f))*sqr(ol_x));
        flttype n1=2*d*f*sqr(x);
        flttype n0=sqr(f*x);

        {
          solver.n0 = n0;
          solver.n1 = n1;
          solver.n2 = n2;
          solver.n3 = n3;
          solver.n4 = n4;
         
          solver.ell_p1 = ol_x;
          solver.ell_p2 = (ol_y-il_y+dzl_y);
          solver.ell_p3 = f;
          solver.ell_p4 = A2;
          solver.ell_p5 = il_y;
          
          solver.ell_x = x;
          solver.ell_y = y;
          
          solver.solve();

          t = solver.res;
        }

        if (t > 2*deltat && t < 1) /*{{{*/
        {
          // zeigSchaleUnten(t, std::cout);
          //
          // Tangente an die Ellipse:
          a  = t*ol_x;
          b  = t*solver.ell_p2 + solver.ell_p3;
          flttype yp = y - t*A2-il_y;

          flttype d1 = b*b*x;
          flttype d2 = a*a*yp;

          
          if (fabs(d1) > fabs(d2))
          {
            dy = 1;
            dx = a*a * (b*b-(yp+dy)*yp) / (b*b*x) - x;
          }
          else
          {
            dx = 1;
            dy = b*b * (a*a-(x+dx)*x) / (a*a*yp) - yp;
          }
          
          if ((x>0 && dy>0) || (x<0 && dy<0) )
          {
            dx *= -1;
            dy *= -1;
          }

          len = sqrt(dx*dx+dy*dy);
          dx  = dx/len;
          dy  = dy/len;

          // senkrecht dazu ist
          {
            // die Ellipse im richtigen Abstand
            t += deltat;
            a  = t*ol_x;
            b  = t*solver.ell_p2 + solver.ell_p3;
            yoff = t*A2+il_y;
            // Schnittpunkt zwischen der Ellipse und der Senkrechten            
            flttype ma, mb, p, q;
            
            // ich will die senkrechte:
            // jetzt rechnen:
            dxs = -dy;
            dys = dx;
            p = (-2*sqr(a)*dys*yoff+2*sqr(a)*dys*y+2*sqr(b)*dxs*x)/(sqr(a*dys)+sqr(b*dxs));
            q = ( sqr(a*yoff)-2*sqr(a)*y*yoff+sqr(a*y)+sqr(b*x)-sqr(a*b) ) / (sqr(a*dys)+sqr(b*dxs));
            
            ma = -p/2 - sqrt(sqr(p)/4 - q);
            mb = -p/2 + sqrt(sqr(p)/4 - q);
            
            if (fabs(mb) < fabs(ma))
              ma = mb;
              
            dxs = dxs*ma;
            dys = dys*ma;
          }
        }
/*}}}*/

      }
    }
    /*}}}*/

    else if (l>1)     /*{{{*/
    {
      // Es könnte im Bereich der oberen Ellipse liegen. Für t=1 gilt
      flttype a    = ou_x;
      flttype b    = 1-ou_y;
      flttype yoff = ou_y;

      if (sqr(x/a)+ sqr((y-yoff)/b) < 1)
      {
        // Es liegt im Bereich der oberen Ellipse. Wie ist t?
        flttype c = dzu_y - iu_y;
        flttype d = 1-dzu_y-ou_y+iu_y;
        flttype f = ou_y-iu_y;

        flttype n4=(sqr(f)-sqr(d))*sqr(ou_x);
        flttype n3=((2*f*iu_y-2*c*d)*sqr(ou_x)-2*f*sqr(ou_x)*y);
        flttype n2=(sqr(ou_x*y)-2*iu_y*sqr(ou_x)*y+sqr(d*x)+(sqr(iu_y)-sqr(c))*sqr(ou_x));
        flttype n1=2*c*d*sqr(x);
        flttype n0=sqr(c*x);

        {
          solver.n0 = n0;
          solver.n1 = n1;
          solver.n2 = n2;
          solver.n3 = n3;
          solver.n4 = n4;
          
          solver.ell_p1 = ou_x;
          solver.ell_p2 = d;
          solver.ell_p3 = c;
          solver.ell_p4 = f;
          solver.ell_p5 = iu_y;
          
          solver.ell_x = x;
          solver.ell_y = y;
          
          solver.solve();
          
          t = solver.res;
        }
                
        if (t >= 2*deltat && t < 1) /*{{{*/
        {
          // zeigSchaleUnten(t, std::cout);
          //
          // Tangente an die Ellipse:
          a  = t * ou_x;
          b  = t * d + c;
          flttype yp = y - yoff;

          flttype d1 = b*b*x;
          flttype d2 = a*a*yp;

          if (fabs(d1) > fabs(d2))
          {
            dy = 1;
            dx = a*a * (b*b-(yp+dy)*yp) / (b*b*x) - x;
          }
          else
          {
            dx = 1;
            dy = b*b * (a*a-(x+dx)*x) / (a*a*yp) - yp;
          }

          if ((x>0 && dy>0) || (x<0 && dy<0) )
          {
            dx *= -1;
            dy *= -1;
          }

          len = sqrt(dx*dx+dy*dy);
          dx  = dx/len;
          dy  = dy/len;
          
          // senkrecht dazu ist
          {
            // die Ellipse im richtigen Abstand
            t += deltat;
            a  = t*ou_x;
            b  = c+t*d;
            yoff = (f*t+iu_y);
            // Schnittpunkt zwischen der Ellipse und der Senkrechten            
            flttype ma, mb, p, q;
            
            // ich will die senkrechte:
            // jetzt rechnen:
            dxs = -dy;
            dys = dx;
            p = (-2*sqr(a)*dys*yoff+2*sqr(a)*dys*y+2*sqr(b)*dxs*x)/(sqr(a*dys)+sqr(b*dxs));
            q = ( sqr(a*yoff)-2*sqr(a)*y*yoff+sqr(a*y)+sqr(b*x)-sqr(a*b) ) / (sqr(a*dys)+sqr(b*dxs));
            
            ma = -p/2 - sqrt(sqr(p)/4 - q);
            mb = -p/2 + sqrt(sqr(p)/4 - q);
            
            if (fabs(mb) < fabs(ma))
              ma = mb;
              
            dxs = dxs*ma;
            dys = dys*ma;
          }
        }
/*}}}*/

      }
    }
/*}}}*/

  }
}
/*}}}*/

void ThermikSchale::showpara() /*{{{*/
{
  std::cout << "    dzu_y = " << dzu_y << "\n";
  std::cout << "    dzl_y = " << dzl_y << "\n";

  std::cout << "    iu_y = " << iu_y << "\n";
  std::cout << "    il_y = " << il_y << "\n";

  std::cout << "    ol_x = " << ol_x << "\n";
  std::cout << "    ol_y = " << ol_y << "\n";

  std::cout << "    ou_x = " << ou_x << "\n";
  std::cout << "    ou_y = " << ou_y << "\n";
}
/*}}}*/

void ThermikSchale::zeigSchaleUnten(flttype t, std::ostream& out) /*{{{*/
{
  flttype a = ol_x * t;
  flttype b = t*(ol_y-il_y+dzl_y)+il_y-dzl_y;
  flttype yoffset = t*(ol_y-il_y)+il_y;
  flttype x, y;

  for (flttype p=0; p<M_PI/2; p+= 0.01)
  {
    x = a * cos(p);
    y = yoffset - b * sin(p);

    parent->zeigPunkt(x, y, out);
  }
  out << "\n\n";
}
/*}}}*/

void ThermikSchale::zeigSchaleOben(flttype t, std::ostream& out) /*{{{*/
{
  flttype a = ou_x * t;
  flttype b = dzu_y - iu_y + t*(1-dzu_y-ou_y+iu_y);
  flttype yoffset = t*(ou_y-iu_y)+iu_y;
  flttype x, y;

  for (flttype p=0; p<M_PI/2; p+= 0.01)
  {
    x = a * cos(p);
    y = yoffset + b * sin(p);

    parent->zeigPunkt(x, y, out);
  }
  out << "\n\n";
}
/*}}}*/

void ThermikSchale::zeigGerade(flttype t, std::ostream& out) /*{{{*/
{
  flttype p_x0 = t*ol_x;
  flttype p_x1 = t*ou_x;

  flttype p_y0 = il_y + t*(ol_y - il_y);
  flttype p_y1 = t*(ol_y - il_y) + iu_y + t*(ou_y - iu_y - ol_y +il_y);

  parent->zeigPunkt(p_x0, p_y0, out);
  parent->zeigPunkt(p_x1, p_y1, out);
  out << "\n\n";
}
/*}}}*/


void ThermikSchale::zeigVectSchaleUnten(flttype t, std::ostream& out) /*{{{*/
{
  flttype a = ol_x * t;
  flttype b = t*(ol_y-il_y+dzl_y)+il_y-dzl_y;
  flttype yoffset = t*(ol_y-il_y)+il_y;
  flttype x, y;
  flttype dx, dy;
  flttype step;
  flttype dxs, dys;
  flttype par_t;

  step = 0.01 + (1-t)*0.1;

  for (flttype p=0; p<M_PI/2; p+= step)
  {
    x = a * cos(p);
    y = yoffset - b * sin(p);

    parent->zeigPunkt(x, y, out);
    getDir(x, y, dx, dy, dxs, dys, par_t);
    parent->zeigPunkt(x+anzfact*dx, y+anzfact*dy, out);
    out << "\n\n";

    
    parent->zeigPunkt(x, y, out);
    parent->zeigPunkt(x+anzfacts*dxs, y+anzfacts*dys, out);
    out << "\n\n";     
  }
}
/*}}}*/

void ThermikSchale::zeigVectSchaleOben(flttype t, std::ostream& out) /*{{{*/
{
  flttype a = ou_x * t;
  flttype b = dzu_y - iu_y + t*(1-dzu_y-ou_y+iu_y);
  flttype yoffset = t*(ou_y-iu_y)+iu_y;
  flttype x, y;
  flttype dx, dy, dxs, dys;
  flttype step;
  flttype par_t;

  step = 0.01 + (1-t)*0.1;

  for (flttype p=0; p<M_PI/2; p+= step)
  {
    x = a * cos(p);
    y = yoffset + b * sin(p);

    parent->zeigPunkt(x, y, out);
    getDir(x, y, dx, dy, dxs, dys, par_t);
    parent->zeigPunkt(x+anzfact*dx, y+anzfact*dy, out);
    out << "\n\n";

    parent->zeigPunkt(x, y, out);
    parent->zeigPunkt(x+anzfacts*dxs, y+anzfacts*dys, out);
    out << "\n\n";       
  }
}
/*}}}*/

void ThermikSchale::zeigVectGerade(flttype t, std::ostream& out) /*{{{*/
{
  flttype step = 0.01/sqrt(sqr(ou_x-ol_x) + sqr(ou_y-ol_y));
  flttype x, y, dx, dy, dxs, dys;
  flttype par_t;

  for (flttype l=0; l<1; l+= step)
  {
    x = t*ol_x + l*t *(ou_x - ol_x);
    y = il_y + t*(ol_y - il_y) + l*  (iu_y -il_y + t*(ou_y - iu_y - ol_y +il_y));

    parent->zeigPunkt(x, y, out);
    getDir(x, y, dx, dy, dxs, dys, par_t);
    parent->zeigPunkt(x+anzfact*dx, y+anzfact*dy, out);
    out << "\n\n";

    parent->zeigPunkt(x, y, out);
    parent->zeigPunkt(x+anzfacts*dxs, y+anzfacts*dys, out);
    out << "\n\n";
  }
}
/*}}}*/

flttype ThermikSchale::get_A_Ref(flttype t) /*{{{*/
{
  // Es wird vereinfacht (in externen Koordinate):
  //   x1 = (1-t)*0.5*(ol_x+ou_x)
  //   x2 = (1-(t-deltat))*0.5*(ol_x+ou_x)
  // 
  //  A = pi * (x2^2 - x1^2)
  // 
  // (x1^2 - x2^2) = ((1-(t-deltat))*0.5*(ol_x+ou_x))^2 - ((1-t)*0.5*(ol_x+ou_x))^2
  
  //   v * A = const
  
  // pi wird hier und bei A_Ref weggelassen, somit stimmt es wieder.
  return(
         sqr(0.5*(ol_x+ou_x)) * (2+deltat-2*t)*deltat
         );  
}
/*}}}*/


