/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2008 Jens Wilhelm Wulf (original author)
 * Copyright (C) 2005, 2006, 2009 Jan Reucker
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
  

#ifndef MOUSE_KBD_H
# define MOUSE_KBD_H

#include <SDL.h>

#include "defines.h"
#include "mod_misc/SimpleXMLTransfer.h"
#include "mod_inputdev/inputdev.h"



/**
 * This class holds information about bindings of certain inputs, like 
 * axes/buttons of a mouse/joystick or keyboard actions.
 * 
 * It also contains the number of the joystick.
 */
class TInputDev
{
  public:
  
   enum { NUM_BUTTONACTION=8 };
   enum { NOTHING=0, RESUME=1, PAUSE=2, RESET=3, INCTHROTTLE=4, DECTHROTTLE=5, ZOOMIN=6, ZOOMOUT=7 };

   enum { NUM_ZOMCONTROL=2 };
   enum { KEYBOARD=0, MOUSE=1};

   TInputDev();
   
   int mouse_bind_x;
   int mouse_bind_y;
   int mouse_bind_l;
   int mouse_bind_m;
   int mouse_bind_r;
   int mouse_bind_u;
   int mouse_bind_d;
   
   int joystick_bind_b[MAXJOYBUTTON + 1];  // joystick buttons
   int joystick_n;                         // sdl joystick num
   
   char** AxisStringsXML;
   char** AxisStringsGUI;
   char** InputMethodStrings;//for XML
   char** InputMethodStringsGUI;// The same but possibly translated
   char** ActionButtonStrings;//for XML
   char** ActionButtonStringsGUI;
   char** MixerStringsXML;
   char** MixerStringsGUI;
   char** ZoomControlStrings;//for XML
   char** ZoomControlStringsGUI;
   /**
    * What input device controls the field of view
    */
   int zoom_control;
      
   /**
    * Loads configuration from <code>cfgfile</code>
    */
   void init(SimpleXMLTransfer* cfgfile);
   
   /**
    * Writes current configuration back into <code>cfgfile</code>
    */
   void putBackIntoCfg(SimpleXMLTransfer* cfgfile);
   
   /**
    * Tries to open the joystick joystick_n. If a joystick is
    * already open, it will be closed before trying to open
    * the new joystick. 
    *
    * \return empty string on success, error message on failure
    */
   std::string openJoystick();
   
   /**
    * Sets joystick_n to joy_n and tries to open this joystick.
    * If a joystick is already open, it will be closed before trying
    * to open the new joystick.
    *
    * \param joy_n number of the joystick to open
    * \return empty string on success, error message on failure
    */
   std::string openJoystick(int joy_n);
   
   /**
    * Closes joystick if it has been opened before.
    */
   void closeJoystick();
   
   /**
    * Get the number of axes of the current joystick.
    * Joystick must have been opened before!
    * \return Number of axes, 0 if joystick is inactive.
    */
   int getJoystickNumAxes();
    
   /**
    * Get the number of buttons of the current joystick.
    * Joystick must have been opened before!
    * \return Number of buttons, 0 if joystick is inactive.
    */
   int getJoystickNumButtons();

  private:   
   int  getValAxis  (std::string asString, int nDefault);
   int  getValButton(std::string asString, int nDefault);
   
   SDL_Joystick* joy;
};

#endif
