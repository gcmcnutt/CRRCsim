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
  

#ifndef CRRC_SLIDER_H
#define CRRC_SLIDER_H

#include <plib/pu.h>

/**
 * This slider differs from puaSliderWithInput in the graphical layout and 
 * in that it checks for min and max values when entering numerical values.
 */
class crrcSlider : public puGroup
{
//  UL_TYPE_DATA

protected:
  puSlider *slider ;
  puInput *input_box ;
  int input_position ;

  static void handle_slider ( puObject *obj ) ;
  static void handle_input ( puObject *obj ) ;
  static void input_down_callback ( puObject *obj ) ;

  void update_widgets ( void ) ;

public:
  // For internal use only:
  void __setInputBox ( float f ) { input_box->setValue ( f ) ; }

  // For public use:

  void draw ( int dx, int dy ) ;
  int  checkHit ( int button, int updown, int x, int y )
  {
    return puGroup::checkHit ( button, updown, x, y ) ;
  }

  int  checkKey ( int key, int updown ) ;

  crrcSlider ( int minx, int miny, int maxx, int maxy, int inputw ) ;

  void setSize ( int w, int h ) ;
   
  void setSliderFraction(float f) { slider->setSliderFraction(f); };

  void setMaxValue ( float f ) { slider->setMaxValue ( f ) ; }
  float getMaxValue ( void ) const { return slider->getMaxValue () ; }
  void setMinValue ( float f ) { slider->setMinValue ( f ) ; }
  float getMinValue ( void ) const { return slider->getMinValue () ; }

  void setValue ( int i ) { slider->setValue ( i ) ;  input_box->setValue ( i ) ; }
  void setValue ( float f ) { slider->setValue ( f ) ;  input_box->setValue ( f ) ; }
  virtual void setValue ( const char *s ) { slider->setValue ( s ) ; }
  virtual void setValue ( bool b ) { slider->setValue ( b ) ; }

  int   getIntegerValue ( void ) { return slider->getIntegerValue () ; }
  float getFloatValue ( void )   { return slider->getFloatValue ()   ; }
  char  getCharValue ( void )    { return slider->getCharValue ()    ; }
  char *getStringValue ( void )  { return slider->getStringValue ()  ; }
  bool  getBooleanValue ( void ) { return slider->getBooleanValue () ; }

  char *getValidData ( void ) const { return input_box->getValidData () ; }
  void setValidData ( char *data ) { input_box->setValidData ( data ) ; }
  void addValidData ( char *data ) { input_box->addValidData ( data ) ; }

  void setCBMode ( int m ) { slider->setCBMode ( m ) ; }
  int getCBMode ( void ) const { return slider->getCBMode () ; }

  void setDelta ( float f ) { slider->setDelta ( f ) ; }
  float getDelta ( void ) const { return slider->getDelta () ; }

  float getStepSize ( void ) const { return slider->getStepSize () ; }
  void setStepSize ( float f )     { slider->setStepSize ( f )     ; }
  
  void  greyOut (void) { slider->greyOut ( ); }

};


#endif
