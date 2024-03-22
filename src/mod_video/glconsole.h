/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2007,2009 Jan Reucker (original author)
 * Copyright (C) 2008 Jens Wilhelm Wulf
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
  

/** \file glconsole.h
 *
 *  This file defines the interface of the GlConsole class
 *  which implements an output console for OpenGL.
 *
 *  \author Jan Reucker (slowhand_47@gmx.de)
 */

#ifndef GLCONSOLE_H_
#define GLCONSOLE_H_

#include <string>
#include <list>

#include <plib/ul.h>
#include <plib/fnt.h>

#include "../mod_main/EventDispatcher.h"


/**
 * A text output console for OpenGL and PLIB
 * 
 * This class implements a text output console.
 *
 * Basic usage:
 * - instanciate a console at a given position and size
 * - call render() once each frame
 * - send text to the console using print()
 */
class GlConsole : public EventListener
{
  public:
    /** default constructor */
    GlConsole(int xsize = 400, int ysize = 100, int xorig = 0, int yorig = 0);
    
    /** destructor */
    ~GlConsole();

    /** render the console */
    void render(int window_width, int window_height);
    
    /** add a line */
    void print(std::string theText);
    
    /** setup the AutoHide feature */
    void setAutoHide(unsigned long display_time_ms, unsigned long fade_time_ms = 1000);
    
    /** show the console */
    void show();
    
    /** hide the console without fading */
    void hide();
    
    /** fade out the console */
    void fadeOut();
    
    /** set the color of the console background */
    void setBGColor(float r, float g, float b, float a);
    
    /** set the color of the console text */
    void setTextColor(float r, float g, float b, float a);
    
    /** interface to the EventDispatcher */
    void operator()(const Event* ev);
    
  private:
    /** possible states of the console */
    typedef enum {HIDDEN, VISIBLE, FADING} CONSOLE_STATE_T;
    
    std::list<std::string> lines;  ///< This list stores the data to be displayed, line by line.
    ulClock   clk;                ///< internal timer
    double    lastActivity;       ///< time of last activity (new lines added...)
    
    // members that control the AutoHide feature
    unsigned long display_time;   ///< time that the console will be visible (in ms), 0 = always visible if not hidden (no AutoHide)
    unsigned long fade_time;      ///< duration of the fade-out (in ms)

    // members for the internal state machine
    CONSOLE_STATE_T state;        ///< state of the console
    double          stateTimer;   ///< timer to control state changes
    
    // members for rendering
    int   size_x;                 ///< horizontal size in pixels
    int   size_y;                 ///< vertical size in pixels
    int   pos_x;                  ///< horizontal position of lower left corner, from left screen border
    int   pos_y;                  ///< vertical position of lower left corner, from lower screen border
    
    int   vspace;                 ///< pixels between two lines
    int   inner_border;           ///< border inside the background rectangle
    unsigned int   visibleLines;  ///< maximum number of visible lines of text
    
    float  relativeOpacity;       ///< current opacity as a fraction of the opacity when fully visible
    double fadeStarted;           ///< record the moment in time when fading started
    
    fntRenderer   fontRenderer;   ///< font renderer to display the text
    
    float   text_r;               ///< text color, red component
    float   text_g;               ///< text color, green component
    float   text_b;               ///< text color, blue component
    float   text_a;               ///< text color, alpha channel (transparency)
    
    float   bg_r;                 ///< background color, red component
    float   bg_g;                 ///< background color, green component
    float   bg_b;                 ///< background color, blue component
    float   bg_a;                 ///< background color, alpha channel (transparency)

 
    /** setup the OpenGL-state for console rendering */
    void setOpenGLState(int w, int h);

    /** restore the original OpenGL state */
    void restoreOpenGLState(void);
    
    /** record any console activity */
    void recordActivity();
    
    /** update the internal timer */
    void updateTimer();

    /** handle the internal state machine */
    void stateMachine();
    
    /** internally add a line to the console */
    void addLine(std::string theLine);
};

#endif /*GLCONSOLE_H_*/
