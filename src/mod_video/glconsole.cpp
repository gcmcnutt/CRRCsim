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
  

/** \file glconsole.cpp
 *
 *  This file implements an output console for OpenGL.
 *
 *  \author Jan Reucker (slowhand_47@gmx.de)
 */
 
#include "../include_gl.h"
#include "glconsole.h"
#include "../mod_misc/filesystools.h"
#include "fonts.h"
#define GLCONSOLE_FONT_FILE  "textures/Helvetica_iso8859-15.txf"

/**
 * Create a GlConsole.
 *
 * \param xsize     horizontal size in pixels
 * \param ysize     vertical size in pixels
 * \param xorig     horizontal position of lower left corner
 * \param yorig     vertical position of lower left corner
 */
GlConsole::GlConsole(int xsize, int ysize, int xorig, int yorig)
 : EventListener(Event::Logging),
   lastActivity(0.0), display_time(0), fade_time(0), state(VISIBLE),
   stateTimer(0.0), size_x(xsize), size_y(ysize), pos_x(xorig), pos_y(yorig),
   vspace(2), inner_border(5), fadeStarted(0.0), fontRenderer(),
   text_r(1.0), text_g(1.0), text_b(1.0), text_a(1.0),
   bg_r(0.2), bg_g(0.2), bg_b(0.2), bg_a(1.0)
{
  clk.reset();

if (Video::textureFont) fontRenderer.setFont(Video::textureFont);
else fontRenderer.setFont(fntGetBitmapFont(FNT_BITMAP_8_BY_13));

  fontRenderer.setPointSize(15);

  visibleLines = (unsigned int)((size_y - (2 * inner_border)) / (fontRenderer.getPointSize() + vspace));
}


/**
 * destructor
 */
GlConsole::~GlConsole()
{
}

/**
 * Renders the console using the current OpenGL context. The console text is
 * stored in a linked list with the most current entry at the head of the list.
 * Therefore the list is walked head-to-tail while rendering the strings
 * from bottom to top until all the vertical space inside the console area is
 * filled.
 * 
 * \param window_width    current OpenGL window width
 * \param window_height   current OpenGL window height
 */
void GlConsole::render(int window_width, int window_height)
{
  std::list<std::string>::iterator it;
  
  stateMachine();
  
  if (state != HIDDEN)
  {
    setOpenGLState(window_width, window_height);
    
    // render the background quad
    glBegin(GL_QUADS);
    glColor4f(bg_r, bg_g, bg_b, bg_a * relativeOpacity);
    glVertex2i(pos_x,           pos_y);
    glVertex2i(pos_x + size_x,  pos_y);
    glVertex2i(pos_x + size_x,  pos_y + size_y);
    glVertex2i(pos_x,           pos_y + size_y);
    glEnd();
    
    // render the text
    glDisable(GL_BLEND);
    fontRenderer.begin();
    glColor4f(text_r, text_g, text_b, text_a * relativeOpacity);
    int i = 0;
    for (it = lines.begin(); it != lines.end(); it++)
    {
      fontRenderer.start2f( pos_x + inner_border,
                            pos_y + inner_border + i * (fontRenderer.getPointSize() + vspace));
      fontRenderer.puts(it->c_str());
      i++;
    }
    fontRenderer.end();
    glEnable(GL_BLEND);
    
    restoreOpenGLState();
  } // state != HIDDEN
}
    
/**
 * The internal state machine of the console.
 */
void GlConsole::stateMachine()
{
  updateTimer();

  switch (state)
  {
    case VISIBLE:
      relativeOpacity = 1.0;
      if (stateTimer < 0)
      {
        fadeOut();
      }
      break;
      
    case FADING:
      relativeOpacity = 1.0 - ((clk.getAbsTime() - fadeStarted) * 1000.0 / (double)fade_time);
      if (relativeOpacity < 0)
      {
        relativeOpacity = 0;
      }
      if (stateTimer < 0)
      {
        hide();
      }
      break;
      
    case HIDDEN:
    default:
      relativeOpacity = 1.0;
      break;
  }
}

/**
 * This method adds new text to the console. Internally the text lines are
 * stored as a linked list of strings. New lines are prepended to the front
 * end of the list. If the string contains newline characters, the string will
 * be recursively split into several lines. Newlines, in general, will not be 
 * stored in the list entries.
 * 
 * \param text    String to be displayed in the console
 */
void GlConsole::print(std::string theText)
{
  std::string::size_type  pos;
  
  if (theText.size() > 0)
  {
    pos = theText.find('\n');    // search for a newline
    if (pos != std::string::npos)
    {
      // found a newline, split the string
      if (pos > 0)
      {
        addLine(theText.substr(0, pos));
        // recurse for the remaining string
        print(theText.substr(pos+1));
      }
    }
    else
    {
      // no newlines, add to list as a whole
      addLine(theText);
    }
  }
}


/**
 * Add a new entry to the list and keep the list size reasonable. Long lines
 * will be split to fit into the console area.
 *
 * \param theLine  Line to be added. Should not contain newlines.
 */
void GlConsole::addLine(std::string theLine)
{
  int maxchars = (size_x - 2 * inner_border) / 8;   // only works for the specified FNT_BITMAP_8_BY_13
  
  do
  {
    lines.push_front(theLine.substr(0, maxchars));
    theLine.erase(0, maxchars);
  } while (theLine.size() > 0);

  recordActivity();
  
  while (lines.size() > visibleLines)
  {
    lines.pop_back();
  }
}


/**
 * Do all steps necessary if there was any kind of activity.
 */ 
void GlConsole::recordActivity()
{
  updateTimer();
  lastActivity = clk.getAbsTime();
  show();
}

/**
 * Update the internal timers. This method should always be called
 * internally if there are any timing-related changes. There should
 * be no explicit calls to clk.update() outside this method, else
 * the stateTimer will get out of sync.
 */
void GlConsole::updateTimer()
{
  clk.update();
  if (stateTimer > 0)
  {
    stateTimer -= clk.getDeltaTime();
  }
}

/** 
 * setup the AutoHide feature 
 */
void GlConsole::setAutoHide(unsigned long display_time_ms, unsigned long fade_time_ms)
{
  display_time = display_time_ms;
  fade_time = fade_time_ms;
}
    
/**
 * show the console 
 */
void GlConsole::show()
{
  state = VISIBLE;
  stateTimer = (double)display_time / 1000.0;
}
    
/**
 * immediately hide the console without fading 
 */
void GlConsole::hide()
{
  state = HIDDEN;
  stateTimer = 0;
}
    
/** 
 * fade out the console 
 */
void GlConsole::fadeOut()
{
  if (state != FADING)
  {
    state = FADING;
    stateTimer = (double)fade_time / 1000.0;
    fadeStarted = clk.getAbsTime();
  }
}


/**
 *  setup OpenGL for console rendering
 */
void GlConsole::setOpenGLState (int w, int h)
{
  glPushAttrib   ( GL_ENABLE_BIT | GL_VIEWPORT_BIT | GL_TRANSFORM_BIT | GL_LIGHTING_BIT ) ;
  glDisable      ( GL_LIGHTING   ) ;
  glDisable      ( GL_FOG        ) ;
  glDisable      ( GL_TEXTURE_2D ) ;
  glDisable      ( GL_DEPTH_TEST ) ;
  glDisable      ( GL_CULL_FACE  ) ;
  glEnable       ( GL_ALPHA_TEST ) ;
  glEnable       ( GL_BLEND ) ;
  glAlphaFunc    ( GL_GREATER, 0.1f ) ;
  glBlendFunc    ( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA ) ;
 
  glViewport     ( 0, 0, w, h ) ;
  glMatrixMode   ( GL_PROJECTION ) ;
  glPushMatrix   () ;
  glLoadIdentity () ;
  gluOrtho2D     ( 0, w, 0, h ) ;
  glMatrixMode   ( GL_MODELVIEW ) ;
  glPushMatrix   () ;
  glLoadIdentity () ;
}


/**
 * restore the original OpenGL state
 */
void GlConsole::restoreOpenGLState ( void )
{
  glMatrixMode   ( GL_PROJECTION ) ;
  glPopMatrix    () ;
  glMatrixMode   ( GL_MODELVIEW ) ;
  glPopMatrix    () ;
  glPopAttrib    () ;
}

/**
 * set the color of the console background 
 *
 * \param r   red component (0.0 ... 1.0)
 * \param g   green component (0.0 ... 1.0)
 * \param b   blue component (0.0 ... 1.0)
 * \param a   alpha channel (opacity) (0.0 ... 1.0)
 */
void GlConsole::setBGColor(float r, float g, float b, float a)
{
  bg_r = r;
  bg_g = g;
  bg_b = b;
  bg_a = a;
}


/**
 * set the color of the console text 
 *
 * \param r   red component (0.0 ... 1.0)
 * \param g   green component (0.0 ... 1.0)
 * \param b   blue component (0.0 ... 1.0)
 * \param a   alpha channel (opacity) (0.0 ... 1.0)
 */
void GlConsole::setTextColor(float r, float g, float b, float a)
{
  text_r = r;
  text_g = g;
  text_b = b;
  text_a = a;
}


/**
 * interface to the EventDispatcher
 *
 * This method is called by the EventDispatcher every time
 * a logging event is raised somewhere in CRRCsim.
 *
 * \param Event   the raised event
 */
void GlConsole::operator()(const Event* ev)
{
  if (ev->getGroup() == Event::Logging)
  {
    const LogMessageEvent *msg = static_cast<const LogMessageEvent*>(ev);
    print(msg->get());
  }
}
