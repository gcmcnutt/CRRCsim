/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2007, 2008 Jens Wilhelm Wulf (original author)
 * Copyright (C) 2005, 2009 Jan Reucker
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
  

#include "zoom.h"

#include "mod_misc/lib_conversions.h"
#include "config.h"

#include <stdio.h>
#include <math.h>

/** global variables -- for description see header file */

float flAutozoom;

/** local variables */

/**
 * field_of_view (without autozoom)
 */
float flFieldOfView;

/**
 * field of view in degrees as calculated by zoom_calc()
 */
float flFieldOfView_Current;


void zoom_in()
{
  flFieldOfView -= 5;
  if (flFieldOfView < 5)
    flFieldOfView = 5;
}

void zoom_out()
{
  flFieldOfView += 5;
  if (flFieldOfView > 65)
    flFieldOfView = 65;
}

void zoom_set(float y)
{
  // limit y within -1..+1
  y = y < -1.0 ? -1.0 : y > 1.0 ? 1.0 : y;
  flFieldOfView = 35 + 30*y;
}

void zoom_reset()
{
  // read values
  flFieldOfView = cfgfile->getDouble("video.zoom.field_of_view", 35);
  flAutozoom    = cfgfile->getDouble("video.zoom.autozoom", 0.05);
}

void zoom_putBackIntoCfg()
{
  cfgfile->setAttributeOverwrite("video.zoom.field_of_view", doubleToString(flFieldOfView));
  cfgfile->setAttributeOverwrite("video.zoom.autozoom",      doubleToString(flAutozoom));
}

float zoom_calc(float flDistance)
{
  const float flNormDist = 21.5;
  float flSizeOfPlane = tan(M_PI/180.0*flFieldOfView) * flNormDist;

  if (flDistance > flNormDist)
    flDistance = flNormDist + (flDistance - flNormDist) * flAutozoom;
  else
    flDistance = flNormDist;
  
  flFieldOfView_Current = 180.0/M_PI*atan(flSizeOfPlane / flDistance);
  
  return(flFieldOfView_Current);
}

float zoom_get()
{
  return(flFieldOfView_Current);
}
