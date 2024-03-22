/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2008 Olivier Bordes (original author)
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

/*
 * General interface for libraries
 */
#ifndef EVENTHANDLER_H
#define EVENTHANDLER_H

#include <iostream>
#include <string>
#include <SDL.h>
#include "../mod_misc/scheduler.h"

using namespace std;

class EventHandler : public RunnableObject 
{
  public:
    EventHandler(Scheduler *_s) ;
    ~EventHandler();
    void Run();
  private:
    void handle_events();
    void mouse_motion(SDL_MouseMotionEvent *event);
    void joystick_motion(SDL_JoyAxisEvent *event);
    void joystick_button(SDL_JoyButtonEvent *event);
    void mouse_button(SDL_MouseButtonEvent *event);
};

#endif
