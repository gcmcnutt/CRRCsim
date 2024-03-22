/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2004, 2005, 2007, 2008 Jens Wilhelm Wulf (original author)
 * Copyright (C) 2004, 2005, 2006, 2007 Jan Reucker
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
  

#ifndef DEFINES_H
#define DEFINES_H

#ifndef TRUE
 #define TRUE 1
#endif

#ifndef FALSE
 #define FALSE 0
#endif

// exit codes for the command line
#define CRRC_EXIT_SUCCESS   (0)
#define CRRC_EXIT_FAILURE   (1)


/*****************************************************************************/

#define MAXJOYAXIS        7    // highest joystick axis allowed (first = 0)
#define MAXJOYBUTTON      9    // highest joystick button allowed (first = 0)

/**
 * Key to zoom in
 */
#define KEY_ZOOM_IN  SDLK_KP_PLUS
//#define KEY_ZOOM_IN  SDLK_e

/**
 * Key to zoom out
 */
#define KEY_ZOOM_OUT  SDLK_KP_MINUS
//#define KEY_ZOOM_OUT  SDLK_i

/**
 * Key for more throttle
 */
#define KEY_THROTTLE_MORE SDLK_PAGEUP
//#define KEY_THROTTLE_MORE SDLK_h

/**
 * Key for less throttle
 */
#define KEY_THROTTLE_LESS SDLK_PAGEDOWN
//#define KEY_THROTTLE_LESS SDLK_a


/**
 * Define to >0 to test joystick configuration without having a joystick
 * (simulated joysticks). The actual number defines how many joysticks
 * are simulated.
 */
#define TEST_WITHOUT_JOYSTICK       0
#define SIMULATED_JOYSTICK_AXES     8   ///< number of axes for simulated joysticks
#define SIMULATED_JOYSTICK_BUTTONS  4   ///< number of buttons for simulated joysticks

#endif
