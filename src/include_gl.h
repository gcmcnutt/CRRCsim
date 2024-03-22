/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2008 Jens Wilhelm Wulf (original author)
 * Copyright (C) 2007 Jan Reucker
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
  

/**
 * Include file for GL, as this is somewhat operating system dependent...
 */

#ifndef INCLUDE_GL_H
# define INCLUDE_GL_H


# ifdef WIN32
#  include <windows.h>
#  include <GL/glu.h>
#  include <GL/glext.h>
# endif


# if defined(__APPLE__) || defined(MACOSX)
#  include <OpenGL/glu.h>
# endif  // __APPLE__


# ifdef linux
#  include <GL/glu.h>
# endif


#endif
