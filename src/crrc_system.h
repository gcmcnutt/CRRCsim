/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2007 Jan Reucker (original author)
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
  

/*****************************************************************************
Title: CRRCsim, the Charles River Radio Control Club Flight Simulator Project
Authors:
Jan Kansky:  Programming
Mark Drela:  Aerodynamics

Purpose:  The idea for CRRCsim is to take away any last excuse you might
have for not using a flight simulator to keep your thumbs certified for RC
flying.   It's free, it works, enjoy.  This is an open source project, so if
you don't like the way something works, help us fix it!

Thanks:
  Bruce Jackson for the LaRCsim framework.
  Flight Gear project for the sky sphere.

Contacts:
If you'd like to help with CRRCSIM, then send me an email!
    email                : kansky@ll.mit.edu
*****************************************************************************/

/** \file crrc_system.h
 *
 *  This file contains the declarations of some platform-specific stuff for
 *  system interaction.
 *
 *  \author Jan Reucker
 */

#ifndef CRRC_SYSTEM_H
#define CRRC_SYSTEM_H

#include <string>

/// The type of a message box
typedef enum {SM_INFO, SM_ERROR, SM_WARNING} SM_TYPE;

/// Send a message to the operating system
void SystemMessage(const char * msg, SM_TYPE type = SM_ERROR);

/// Get a string containing the operating system name and version
char * getOSVersionString();

/// Get the current system time as a formatted string
void getSystemTimeString(std::string& t);

#endif  // CRRC_SYSTEM_H
