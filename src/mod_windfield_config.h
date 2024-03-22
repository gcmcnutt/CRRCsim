/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2008 Jens Wilhelm Wulf (original author)
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
  

#ifndef MOD_WINDFIELD_CONFIG_H
# define MOD_WINDFIELD_CONFIG_H

/**
 * Which thermal simulation to use?
 *  0: the 'old' code 
 *  1: Jens Wilhelm Wulf, January 2005
 * There is a document in 'documentation' describing both of them.
 */
#define THERMAL_CODE 1

/**
 * Define as 1 to be able to take a picture of the windfield.
 * Press 's' when flying to save current state.
 * See windfield.cpp
 */
#define DEBUG_THERMAL_SCRSHOT 1

#endif
