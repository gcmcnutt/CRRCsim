/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2008,2009 Jan Reucker (original author)
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
  

/** \file gloverlay.cpp
 *
 *  This file contains the implementation of GlOverlay.
 *
 *  \author Jan Reucker (slowhand_47@gmx.de)
 */

#include "gloverlay.h"
#include "../include_gl.h"

#ifdef DEBUG_GLOVERLAY
#include <iostream>
#endif

/// stores the horizontal window size
int GlOverlay::window_size_x = 640;
/// stores the vertical window size
int GlOverlay::window_size_y = 480;

/// stores pointers to all instances
std::list<GlOverlay*> GlOverlay::GlOverlayInstances;

/// the constructor
GlOverlay::GlOverlay()
{
  registerInstance(this);
}

/// the destructor
GlOverlay::~GlOverlay()
{
  unregisterInstance(this);
}

void GlOverlay::registerInstance(GlOverlay *ovl)
{
  GlOverlayInstances.push_back(ovl);
#ifdef DEBUG_GLOVERLAY
  std::cout << "GlOverlay: registered instance " << ovl;
  std::cout << "(total: " << GlOverlayInstances.size() << "instances)" << std::endl;
#endif
}

void GlOverlay::unregisterInstance(GlOverlay *ovl)
{
  std::list<GlOverlay*>::iterator it;
  bool found = false;
  
  for (it = GlOverlayInstances.begin();
       (it != GlOverlayInstances.end()) && !found;
       it++)
  {
    if (*it == ovl)
    {
      found = true;
    }
  }
  if (found)
  {
    GlOverlayInstances.erase(it);
#ifdef DEBUG_GLOVERLAY
    std::cout << "GlOverlay: unregistered instance " << ovl << std::endl;
#endif
  }
  else
  {
#ifdef DEBUG_GLOVERLAY
    std::cerr << "*** GlOverlay: unable to unregister unmanaged instance " << ovl << std::endl;
#endif
  }
}

/**
 *  This method switches to the 2D rendering state and
 *  viewing transformation.
 *
 */
void GlOverlay::setupRenderingState(int window_xsize, int window_ysize)
{
  glPushAttrib(GL_ALL_ATTRIB_BITS);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);
  glMatrixMode (GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity ();
  gluOrtho2D (0, window_xsize-1, 0, window_ysize);
  glMatrixMode(GL_MODELVIEW);
}


/**
 *  This method switches back to the original rendering state.
 */
void GlOverlay::restoreRenderingState()
{
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
  glMatrixMode (GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopAttrib();
}

/**
 *  Render all instances of GlOverlay
 */
void GlOverlay::renderAllInstances(int window_xsize, int window_ysize)
{
  // store the window size for all derived classes
  window_size_x = window_xsize;
  window_size_y = window_ysize;

  setupRenderingState(window_xsize, window_ysize);
  
  std::list<GlOverlay*>::iterator it;
  
  for (it = GlOverlayInstances.begin(); it != GlOverlayInstances.end(); it++)
  {
    (*it)->draw();
  }
  
  restoreRenderingState();
}
