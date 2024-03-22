/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2008 Jens Wilhelm Wulf (original author)
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
  

#ifndef CRRC_SPIN_H
#define CRRC_SPIN_H

#include <plib/pu.h>

/**
 * This slider differs from puaSpinBox in that it doesn't
 * allow keyboard input and has bigger Buttons. It is made for input
 * of integers only.
 */
class crrcSpin : public puGroup
{
//  UL_TYPE_DATA

  protected :
   puInput       *input_box ;
   puArrowButton *up_arrow ;
   puArrowButton *down_arrow ;
   int           nValue;
   int           nValueMin;
   int           nValueMax;
   

  public:
   crrcSpin ( int minx, int miny, int maxx, int maxy);
   
   void setMinValue(int i) { nValueMin = i; };
   void setMaxValue(int i) { nValueMax = i; };

   int getMinValue() { return(nValueMin); };
   int getMaxValue() { return(nValueMax); };
   
   int getIntValue() { return(nValue); };

   void setValue (int i);
};
#endif
