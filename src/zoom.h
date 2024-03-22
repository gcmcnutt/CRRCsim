/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2007, 2008 Jens Wilhelm Wulf (original author)
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
  

#ifndef ZOOM_H
#define ZOOM_H

/**
 * Apply automatic zoom?
 * 0: no automatic zoom
 * 1: full automatic zoom (constant visual size of plane)
 */
extern float flAutozoom;

/**
 * Reset field of view and flAutozoom to initial value
 */
void zoom_reset();

/**
 * Write field of view and flAutozoom back to cfg
 */
void zoom_putBackIntoCfg();

void zoom_in();
void zoom_out();
void zoom_set(float y);

/**
 * Automatically adjust the field of view so that the 
 * plane doesn't become too small.
 * Returns the field of view in degrees.
 */
float zoom_calc(float flDistance);

/**
 * Returns field of view in degrees as calculated by last 
 * call of zoom_calc(), i.e. including autozoom effect.
 */
float zoom_get();

#endif
