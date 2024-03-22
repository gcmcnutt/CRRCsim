/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2010 Jan Reucker (original author)
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

/** \file fonts.h
 *
 *  Declaration of enhanced fonts for PLIB.
 */

#ifndef FONTS_H_
#define FONTS_H_

#include <plib/pu.h>

// Declaration of fonts defined in fonts.cpp
extern puFont FONT_HELVETICA_14;
extern puFont FONT_SANS_12B;

namespace Video
{
//Declaration of fonts defined in grapĥics.cpp.
extern fntTexFont    *textureFont; //textured font  
extern std::string        codeSet; //for font filename construct
 
}
#endif  // FONTS_H_

