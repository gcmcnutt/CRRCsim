/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2008 Jan Reucker (original author)
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
  

/** \file gloverlay.h
 *
 *  This file defines the interface of the GlOverlay class
 *  which a base class for 2D overlays in a PLIB rendering
 *  environment.
 *
 *  \author Jan Reucker (slowhand_47@gmx.de)
 */

#ifndef _GLOVERLAY_H
#define _GLOVERLAY_H

#include <list>

#define DEBUG_GLOVERLAY 1

/**
 *  Base class for 2D overlays
 *  
 *  This abstract base class provides basic mechanisms common to all
 *  2D overlays.
 */
class GlOverlay
{
  public:
    /** 
     *  constructor
     */
    GlOverlay();
  
    /**
     *  destructor
     */
    virtual ~GlOverlay();
    
    /**
     *  Draw the overlay
     *
     *  Derived classes must implement this method.
     */
    virtual void draw() const = 0;
  
    /**
     *  Draw all overlays
     */
    static void renderAllInstances(int window_xsize, int window_ysize);

    /**
     *  Switch to the 2D rendering state
     */
    static void setupRenderingState(int window_xsize, int window_ysize);
  
    /**
     *  Switch back to the original rendering state
     */
    static void restoreRenderingState();
  
  protected:
    static int window_size_x;
    static int window_size_y;

  private:
    static std::list<GlOverlay*> GlOverlayInstances;
  
    void registerInstance(GlOverlay *ovl);
    void unregisterInstance(GlOverlay *ovl);
};

#endif    // _GLOVERLAY_H
