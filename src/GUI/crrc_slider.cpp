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
  

#include "crrc_slider.h"
#include <stdio.h>

// UL_RTTI_DEF1(crrcSlider,puGroup)

void crrcSlider::handle_slider ( puObject *obj )
{
  crrcSlider *sl = (crrcSlider *)obj->getUserData () ;
  sl->setValue ( obj->getFloatValue () ) ;
  sl->__setInputBox ( obj->getFloatValue () ) ;

  sl->invokeCallback () ;
}

void crrcSlider::handle_input ( puObject *obj  )
{
  float val;
  float m;
    
  crrcSlider *sl = (crrcSlider *)obj->getUserData () ;
  
  val = obj->getFloatValue();
  
  m = sl->getMaxValue();
  if (val > m)
    val = m;
  m = sl->getMinValue();
  if (val < m)
    val = m;
  
  sl->setValue ( val ) ;
  sl->invokeCallback () ;
}

crrcSlider::crrcSlider ( int minx, int miny, int maxx, int maxy, int inputw ) : puGroup ( minx, miny )
{
  type |= PUCLASS_SLIDER ;
  slider = new puSlider (inputw, 0, maxx-minx-inputw, false, maxy-miny) ;
  input_box = new puInput (0, 0, inputw, maxy-miny) ;
  input_box->setValue ( 0 ) ;
  slider->setUserData ( this ) ;
  slider->setCallback ( handle_slider ) ;
  input_box->setUserData ( this ) ;
  input_box->setCallback ( handle_input ) ;
  close () ;
}


void crrcSlider::setSize ( int w, int h )
{
  slider->setSize ( 20, h-40 ) ;
  slider->setPosition ( w/2-10, 0 ) ;

  input_box->setSize ( w, 20 ) ;
  input_box->setPosition ( 0, 0 ) ;
}


void crrcSlider::draw ( int dx, int dy )
{
  if ( !visible || ( window != puGetWindow () ) ) return ;

  draw_label ( dx, dy ) ;

  puGroup::draw ( dx, dy ) ;
}


int crrcSlider::checkKey ( int key, int updown )
{
  if ( ! isVisible () || ! isActive () || ( window != puGetWindow () ) )
    return FALSE ;

  return ( input_box->checkKey ( key, updown ) ) ;
}

