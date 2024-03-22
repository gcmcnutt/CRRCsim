/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2008 Jens Wilhelm Wulf (original author)
 * Copyright (C) 2008 Jan Reucker
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
  

#include "crrc_spin.h"
#include <stdio.h>
#include "plib/puAux.h"

// UL_RTTI_DEF1(crrcSpin,puGroup)

crrcSpin::crrcSpin ( int minx, int miny, int maxx, int maxy) :
               puGroup ( minx, miny )
{
  extern void crrcSpin_handle_input ( puObject* ob ) ;
  extern void crrcSpin_handle_input_active ( puObject* ob ) ;
  extern void crrcSpin_handle_arrow ( puObject* ob ) ;
  type |= PUCLASS_SPINBOX ;
  
  int arrow_size = maxy - miny;

  input_box = new puInput ( arrow_size, 0, maxx - minx - arrow_size, maxy - miny ) ;
  input_box->disableInput();

  down_arrow = new puArrowButton ( 0, 0,
                                 arrow_size, arrow_size, PUARROW_DOWN ) ;
  down_arrow->setCallback ( crrcSpin_handle_arrow ) ;
  down_arrow->setUserData ( this ) ;

  up_arrow = new puArrowButton (maxx - minx - arrow_size, 0,
                                maxx - minx, arrow_size, PUARROW_UP ) ;
  up_arrow->setCallback ( crrcSpin_handle_arrow ) ;
  up_arrow->setUserData ( this ) ;

  close () ;
}

void crrcSpin_handle_arrow ( puObject *ob ) 
{
  crrcSpin *master = (crrcSpin *)(ob->getUserData ()) ;
  int val = master->getIntValue () ;
  if ( ((puArrowButton *)ob)->getArrowType () == PUARROW_UP )
    val++;
  else
    val--;

  master->setValue (val) ;
  master->invokeCallback () ;
}

void crrcSpin::setValue (int i)
{ 
  if (i > nValueMax) 
    i = nValueMax;
  if (i < nValueMin)
    i = nValueMin;
  nValue = i;  
  input_box->setValue(i); 
};
