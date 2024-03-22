// -*- mode: c; mode: fold -*-
/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2006, 2008 Jens Wilhelm Wulf (original author)
 * Copyright (C) 2006 Jan Reucker
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

#include "tschalen.h"

#include <iostream>
#include <fstream>
#include <math.h>

const flttype VelFact = 0.002;

//
#define DREHEN 1

void ThermikSchalen::trans(flttype& x, flttype& y) /*{{{*/
{
#if (DREHEN != 0)
  flttype x_tmp = flTransCos*x - flTransSin*y + x0;
  y = flTransCos*y + flTransSin*x;
  x = x_tmp;
#endif
}
/*}}}*/

void ThermikSchalen::trans_re(flttype& x, flttype& y) /*{{{*/
{
#if (DREHEN != 0)
  flttype tmp   = x - x0;
  x = flTransReCos*tmp - flTransReSin*y;
  y = flTransReCos*y   + flTransReSin*tmp;
#endif
}
/*}}}*/

void ThermikSchalen::zeigPunkt(flttype x, flttype y, std::ostream& out) /*{{{*/
{
  trans(x, y);
  out << x << " " << y << "\n";
}
/*}}}*/

void ThermikSchalen::createDefaultConfig(SimpleXMLTransfer* cfg) /*{{{*/
{
  /* <v3 vRefExp="5" dz_m="50" height_m="600"> */
  /* <inside> */
  /*   <upper r_m="30" sl_r="0.8" sl_dz_r="1.0" /> */
  /*   <lower r_m="20" sl_r="0.9" sl_dz_r="0.5" /> */
  /* </inside> */
  /* <outside> */
  /*   <upper r_m="65" sl_r="0" sl_dz_r="1" /> */
  /*   <lower r_m="65" sl_r="0" sl_dz_r="1" /> */
  /* </outside> */
  /* </v3> */

  cfg->setAttribute("v3.vRefExp",    "5");
  cfg->setAttribute("v3.dz_m",      "50");
  cfg->setAttribute("v3.height_m", "600");

  cfg->setAttribute("v3.inside.upper.r_m",     "30");
  cfg->setAttribute("v3.inside.upper.sl_r",    "0.8");
  cfg->setAttribute("v3.inside.upper.sl_dz_r", "1.0");
  cfg->setAttribute("v3.inside.lower.r_m",     "20");
  cfg->setAttribute("v3.inside.lower.sl_r",    "0.9");
  cfg->setAttribute("v3.inside.lower.sl_dz_r", "0.5");

  cfg->setAttribute("v3.outside.upper.r_m",     "65");
  cfg->setAttribute("v3.outside.upper.sl_r",    "0");
  cfg->setAttribute("v3.outside.upper.sl_dz_r", "1");
  cfg->setAttribute("v3.outside.lower.r_m",     "65");
  cfg->setAttribute("v3.outside.lower.sl_r",    "0");
  cfg->setAttribute("v3.outside.lower.sl_dz_r", "1");
}
/*}}}*/

void ThermikSchalen::init(SimpleXMLTransfer* cfg) /*{{{*/
{
  double h_m;
  double dz;

  std::cout << "--- void ThermikSchalen::init ---------------------------\n";
  
  h_m = cfg->attributeAsDouble("height_m");

  dz = (h_m - 2*cfg->attributeAsDouble("dz_m")) / h_m;

  if (dz < 0)
  {
    throw XMLException("Thermal config error: dz_m is larger than 0.5*h_m");
  }

  r_max = 0;

  inner.dzu_y = 0.5 + 0.5*dz;
  inner.ou_x  = -1*cfg->getDouble("inside.upper.r_m")/h_m;
  inner.ou_y  = 0.5 + (inner.dzu_y-0.5) * cfg->getDouble("inside.upper.sl_r");
  inner.iu_y  = 0.5 + (inner.dzu_y-0.5) * cfg->getDouble("inside.upper.sl_r") * cfg->getDouble("inside.upper.sl_dz_r");
  
  inner.dzl_y = 0.5 - 0.5*dz;
  inner.ol_x  = -1*cfg->getDouble("inside.lower.r_m")/h_m;
  inner.ol_y  = inner.dzl_y + (0.5-inner.dzl_y) * (1-cfg->getDouble("inside.lower.sl_r"));
  inner.il_y  = inner.dzl_y + (0.5-inner.dzl_y) * (1-cfg->getDouble("inside.lower.sl_r")*cfg->getDouble("inside.lower.sl_dz_r"));
  
  outer.dzu_y = inner.dzu_y;
  outer.ou_x  = cfg->getDouble("outside.upper.r_m")/h_m;
  outer.ou_y  = 0.5 + (outer.dzu_y-0.5) * cfg->getDouble("outside.upper.sl_r");
  outer.iu_y  = 0.5 + (outer.dzu_y-0.5) * cfg->getDouble("outside.upper.sl_r") * cfg->getDouble("outside.upper.sl_dz_r");
  
  outer.dzl_y = inner.dzl_y;
  outer.ol_x  = cfg->getDouble("outside.lower.r_m")/h_m;
  outer.ol_y  = outer.dzl_y + (0.5-outer.dzl_y) * (1-cfg->getDouble("outside.lower.sl_r"));
  outer.il_y  = outer.dzl_y + (0.5-outer.dzl_y) * (1-cfg->getDouble("outside.lower.sl_r")*cfg->getDouble("outside.lower.sl_dz_r"));

  inner.parent = this;
  outer.parent = this;

  inner.init();
  outer.init();
  
  std::cout << "  inner:\n";
  inner.showpara();
  std::cout << "\n  outer:\n";
  outer.showpara();
  
  r_ref = -inner.ol_x;
  std::cout << "\n  r_ref = " << r_ref << "\n";
  
  {
    flttype angle;

    if (inner.ou_x == inner.ol_x)
      angle = 0;
    else
      angle = atan2(inner.ou_x-inner.ol_x, inner.ou_y-inner.ol_y);

    flttype len   = sqrt(inner.ol_x*inner.ol_x + inner.ol_y*inner.ol_y);
    flttype alpha = atan2(inner.ol_y, inner.ol_x) + angle;
    
    x0 = -1 * len * cos(alpha);
    
    flTransSin   = sin(angle);
    flTransCos   = cos(angle);
    flTransReSin = sin(-1*angle);
    flTransReCos = cos(-1*angle);
  }

  {
    flttype x, y;    
    if (outer.ol_x > outer.ou_x)
    {
      x = outer.ol_x;
      y = outer.ol_y;
    }
    else
    {
      x = outer.ou_x;
      y = outer.ou_y;
    }            
    trans(x, y);
    r_max = x;
    std::cout << "  r_max = " << r_max << "\n";
  }
  
  vRefExp = cfg->attributeAsDouble("vRefExp");
    
  {
    std::string filename = cfg->attribute("fileC", "");

    if (filename.length() > 0)
    {
      std::ofstream out;

      out.open(filename.c_str());
      if (!out)
      {
        std::cerr << "Error opening " << filename.c_str() << "\n";
      }
      else
      {        
        flttype x, y, dx, dy;

        for (x=-r_max; x<r_max; x += r_max/100)
        {
          for (y=0; y<1; y += 1.0/300)
          {
            vectorAt(x, y, dx, dy, 1);
            if (sqrt(dx*dx+dy*dy) > 1.0E-5)
            {
              out << x            << " " << y    << "\n";
              out << x+VelFact*dx << " " << y+VelFact*dy << "\n";
              out << "\n";
            }
          }
        }
        
        out.close();
      }
    }
  }

  {
    std::string filename = cfg->attribute("fileB", "");

    if (filename.length() > 0)
    {
      std::ofstream out;

      out.open(filename.c_str());
      if (!out)
      {
        std::cerr << "Error opening " << filename.c_str() << "\n";
      }
      else
      {        
        for (flttype t=0.0499; t<=1; t+= 0.05)
        {
          outer.zeigVectSchaleUnten(t, out);
          outer.zeigVectGerade(t, out);
          outer.zeigVectSchaleOben(t, out);
          inner.zeigVectSchaleUnten(t, out);
          inner.zeigVectGerade(t, out);
          inner.zeigVectSchaleOben(t, out);
        }
        
        out.close();
      }
    }
  }

  {
    std::string filename = cfg->attribute("fileA", "");

    if (filename.length() > 0)
    {
      std::ofstream out;

      out.open(filename.c_str());
      if (!out)
      {
        std::cerr << "Error opening " << filename.c_str() << "\n";
      }
      else
      {        
        for (flttype t=0.125; t<=1; t+= 0.125)
        {
          outer.zeigSchaleUnten(t, out);
          outer.zeigGerade(t, out);
          outer.zeigSchaleOben(t, out);
          inner.zeigSchaleUnten(t, out);
          inner.zeigGerade(t, out);
          inner.zeigSchaleOben(t, out);
        }
        
        out.close();
      }
    }
  }
  
  // Reference velocity is: center velocity at y=0.5. Radius is 0.5*(inner.ou_x+inner.ol_x) there.
  // At the lower end of the straight line, radius is inner.ol_x.
  //   rref=0.5*(inner.ou_x+inner.ol_x)
  //   rl  = inner.ol_x
  // 
  //   vref*pi*(rref/100)^2 = vl*pi*(rl/100)^2
  //   vl = vref*(rref/100)^2 / (rl/100)^2
  //   vl = vref*rref^2 / rl^2
  std::cout << "  Upwards velocity at lower end is " 
    << (0.5*(inner.ou_x+inner.ol_x)*0.5*(inner.ou_x+inner.ol_x) / (inner.ol_x*inner.ol_x))
    << " times upwards velocity at average height.\n";
  
  std::cout << "---------------------------------------------------------\n";
}
/*}}}*/

void ThermikSchalen::vectorAt(flttype  x,  flttype  y,
                              flttype& dx, flttype& dy,
                              flttype  vRef) /*{{{*/
{
  flttype dxs, dys, t;
  
  dx = 0;
  dy = 0;
  
  if (x < r_max && y > 0 && y < 1)
  {
    trans_re(x, y);

    if (x < 0)
    {
      inner.getDir(x, y, dx, dy, dxs, dys, t);
    }
    else
    {
      outer.getDir(x, y, dx, dy, dxs, dys, t);
    }

    flttype len = sqrt(dx*dx+dy*dy);
    
    if (t < 0 || t > 1 || len < 1.0E-3)
    {
      dx = 0;
      dy = 0;
    }
    else
    {
      flttype A_ref = inner.get_A_Ref(t);
      flttype  x1, y1;
    
      x1 = x - dxs;
      y1 = y - dys;
      
      trans(x,  y);
      trans(x1, y1);
      
      // Es gilt die Mantelfläche vom Kegelstumpf.
      flttype l = sqrt((x-x1)*(x-x1) + (y-y1)*(y-y1));
      // pi wird hier und bei A_Ref weggelassen, somit stimmt es wieder.
      flttype A          = l * (x+x1);
      flttype v_RefLocal = vRef * ( 1 - pow(1-t, vRefExp)  );

      // v_RefLocal * A_ref = A * v
      // v = v_RefLocal * A_ref / A
      flttype v = v_RefLocal * A_ref / A;
      flttype leninv = v/len;
           
      dx *= leninv;
      dy *= leninv;
      
      // std::cout << t << " " << A_ref << "\n";
                  
#if (DREHEN != 0)
      flttype dx_tmp;
      dx_tmp = flTransCos*dx - flTransSin*dy;
      dy     = flTransCos*dy + flTransSin*dx;
      dx     = dx_tmp;
#endif
            
    }
  }  
}
/*}}}*/

