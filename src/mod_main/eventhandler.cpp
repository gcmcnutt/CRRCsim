/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2008 Olivier Bordes (original author)
 * Copyright (C) 2009 Jan Reucker
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

#include <iostream>
#include <string>
#include "eventhandler.h"
#include "../global_video.h"
#include "../GUI/crrc_joy.h"
#include "../mod_inputdev/inputdev.h"
#include "../GUI/crrc_gui_main.h"
#include "../global.h"
#include "../mouse_kbd.h"
#include "../SimStateHandler.h"
#include "../zoom.h"

extern void key_down(SDL_keysym *keysym);


EventHandler::EventHandler(Scheduler *_s)
{
  myScheduler= _s;
  _s->Register( this);
}

EventHandler::~EventHandler()
{
  myScheduler->UnRegister( this);
}

void EventHandler::Run()
{
  handle_events();
}

void EventHandler::handle_events()
{
  SDL_Event event;
  while (SDL_PollEvent(&event))
  {
    switch (event.type)
    {
      case SDL_VIDEORESIZE:
        Video::resize_window(event.resize.w, event.resize.h);
        break;

      case SDL_JOYAXISMOTION:
        joystick_motion(&event.jaxis);
        break;

      case SDL_JOYBUTTONDOWN:
        if (nJoystickDlg)
          joystickDlg->joystickDlgButton(&event.jbutton);
        else
          joystick_button(&event.jbutton);
        break;

      case SDL_MOUSEMOTION:
        /**
         * GUI_MOUSE_MOTION_WORKAROUND
         * The GUI sometimes returns that it used the event, although it is not visible anymore!
         * Therefore we need to check whether it is visible at all.
         * Where it works:
         *   -ESC to show GUI
         *   -open dialog (for example video)
         *   -close dialog
         *   -ESC to hide GUI
         * Everything is fine, the mouse-motions are not used by the GUI.
         *   -ESC to show GUI
         *   -toggle something from the menu bar (for example Verbosity)
         *   -ESC to hide GUI
         * Now the GUI still uses the mouse motions, although it is invisible!
         */
        if (Global::TXInterface->inputMethod() == T_TX_Interface::eIM_mouse)
        {
          // always update the mouse interface
          mouse_motion(&event.motion);
          if (Global::gui)
            Global::gui->mouseMotionHandler(event.motion.x, event.motion.y);
        }
        else if (!Global::gui || !Global::gui->isVisible()
            ||
            !Global::gui->mouseMotionHandler(event.motion.x, event.motion.y))
        {
          // GUI did not use the event, pass it to the simulation itself
          mouse_motion(&event.motion);
        }
        break;

      case SDL_MOUSEBUTTONDOWN:
#if TEST_WITHOUT_JOYSTICK > 0
        if (nJoystickDlg && event.button.button != SDL_BUTTON_LEFT)
        {
          SDL_JoyButtonEvent jevent;
          jevent.type   = SDL_JOYBUTTONDOWN;
          jevent.state  = SDL_PRESSED;
          jevent.which  = 0;
          switch (event.button.button)
          {
            case SDL_BUTTON_MIDDLE:
              jevent.button = 0;
              break;
            case SDL_BUTTON_RIGHT:
              jevent.button = 1;
              break;
            default:
              jevent.button = 2;
              break;
          }
          joystickDlg->joystickDlgButton(&jevent);
        }
#endif
        if (!Global::gui || !Global::gui->mouseButtonDownHandler(event.button.button, event.button.x, event.button.y))
        {
          // GUI did not use the event, pass it to the simulation itself
          mouse_button(&event.button);
        }
        break;

      case SDL_MOUSEBUTTONUP:
        if (Global::gui)
          Global::gui->mouseButtonUpHandler(event.button.button, event.button.x, event.button.y);
        break;

      case SDL_KEYDOWN:
        if (!Global::gui || !Global::gui->keyDownEventHandler(event.key.keysym))
        {
          // GUI did not use the event, pass it to the simulation itself
          key_down(&event.key.keysym);
        }
        break;

      case SDL_KEYUP:
        if (Global::gui)
          Global::gui->keyUpEventHandler(event.key.keysym);
        break;

      case SDL_QUIT:
        if (Global::gui)
          Global::gui->doQuitDialog();
        else
          Global::Simulation->quit();
        break;
    }
  }
}


/***************************************************************************/
void EventHandler::mouse_motion(SDL_MouseMotionEvent *event)
{
  int x, y;

  // Not returning in test_mode makes test_mode work with mouse input,
  // but messes up input values a little.
  //~ if (nState != STATE_RUN && !(test_mode && nState == STATE_RESUMING))
  //~ return;

  x = event->x;
  y = event->y;

  if ((Global::inputDev->zoom_control==TInputDev::MOUSE) && !(Global::gui && Global::gui->isVisible()))
  {
    int xsize, ysize;
    Video::getWindowSize(xsize, ysize);
    zoom_set(2.0*(y - (float)ysize/2.0)/(float)ysize);
    return;
  }

  if (Global::TXInterface->inputMethod() == T_TX_Interface::eIM_mouse)
  {
    int xsize, ysize;
    Video::getWindowSize(xsize, ysize);
    Global::TXInterface->setAxis(0, (float)x / (float)xsize - 0.5);
    Global::TXInterface->setAxis(1, (float)y / (float)ysize - 0.5);
  }
}


/*****************************************************************************/
#ifndef NORM_JOYSTICK
# define NORM_JOYSTICK(X) ((float)(X) / 65536.0)
//#define NORM_JOYSTICK(X) (((float)(X) -50 )/ 200 ) // for realflight joystick&linux

#endif
void EventHandler::joystick_motion(SDL_JoyAxisEvent *event)
{
  if (event->axis > MAXJOYAXIS)
    return;

  Global::TXInterface->setAxis(event->axis, NORM_JOYSTICK(event->value));
}

void EventHandler::joystick_button(SDL_JoyButtonEvent *event)
{
  if (event->state != SDL_PRESSED || event->button > MAXJOYBUTTON)
    return;

  switch (Global::inputDev->joystick_bind_b[event->button])
  {
    case TInputDev::RESUME:
      if (!(Global::gui && Global::gui->isVisible()))
      {
        Global::Simulation->resume();
      }
      break;
    
    case TInputDev::RESET:
      if (! CRRCDialog::getToplevel() )
      {
        if (Global::gui && Global::gui->isVisible())
        {
          Global::gui->hide();
        }
        Global::Simulation->reset();
      }
      break;
    
    case TInputDev::PAUSE:
      if (!(Global::gui && Global::gui->isVisible()))
      {
        Global::Simulation->pause();
      }
      break;
    
    case TInputDev::INCTHROTTLE:
      Global::TXInterface->increase_throttle();
      break;
    
    case TInputDev::DECTHROTTLE:
      Global::TXInterface->decrease_throttle();
      break;
    
    case TInputDev::ZOOMIN:
      zoom_in();
      break;

    case TInputDev::ZOOMOUT:
      zoom_out();
      break;
  }
}

/*****************************************************************************/

/**
 *  Handle mouse buttons
 *
 *  This method evaluates any SDL mouse button events and dispatches them
 *  to the configured handler functions.
 *
 *  Note that mouse wheel movement is also reported as button presses.
 *  Scrolling the wheel up generates an SDL_MOUSEBUTTONDOWN event for
 *  SDL_BUTTON_WHEELUP, immediately followed by an SDL_MOUSEBUTTONDOWN event.
 *
 *  \todo Instead of handling the mouse wheel events like buttons, we could
 *        in general handle it like a third axis for the mouse interface
 *        and let the user configure the axis mapping to his liking.
 *
 *  \param event  The pending SDL_MOUSEBUTTONDOWN event.
 */
void EventHandler::mouse_button(SDL_MouseButtonEvent *event)
{
  if (event->state != SDL_PRESSED)
    return;
  
  int mouse_binding = TInputDev::NOTHING;

  switch (event->button)
  {
    case SDL_BUTTON_LEFT:
      mouse_binding = Global::inputDev->mouse_bind_l;
      break;
      
    case SDL_BUTTON_RIGHT:
      mouse_binding = Global::inputDev->mouse_bind_r;
      break;      
      
    case SDL_BUTTON_MIDDLE:
      mouse_binding = Global::inputDev->mouse_bind_m;
      break;
    
#ifndef SDL_WITHOUT_MOUSEWHEEL
    case SDL_BUTTON_WHEELUP:
      mouse_binding = Global::inputDev->mouse_bind_u;
      break;

    case SDL_BUTTON_WHEELDOWN:
      mouse_binding = Global::inputDev->mouse_bind_d;
      break;
#endif
    
    default:
      break;
  }
      
  switch (mouse_binding)
  {
    case TInputDev::RESUME:
      if (!(Global::gui && Global::gui->isVisible()))
      {
        Global::Simulation->resume();
      }
      break;
    
    case TInputDev::RESET:
      if (!(Global::gui && Global::gui->isVisible()))
      {
        Global::Simulation->reset();
      }
      break;
    
    case TInputDev::PAUSE:
      if (!(Global::gui && Global::gui->isVisible()))
      {
        Global::Simulation->pause();
      }
      break;
    
    case TInputDev::INCTHROTTLE:
      Global::TXInterface->increase_throttle();
      break;
    
    case TInputDev::DECTHROTTLE:
      Global::TXInterface->decrease_throttle();
      break;
    
    case TInputDev::ZOOMIN:
      zoom_in();
      break;

    case TInputDev::ZOOMOUT:
      zoom_out();
      break;

    default:
      break;
  }
}


