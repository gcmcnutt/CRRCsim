/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2007-2009 Jan Reucker (original author)
 * Copyright (C) 2007, 2008 Jens Wilhelm Wulf
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
  

/** \file crrc_graphics.h
 *
 *  The public declarations for crrc_graphics.cpp.
 */

#ifndef CRRC_GRAPHICS_H
#define CRRC_GRAPHICS_H

#include <plib/ssg.h>
#include "../include_gl.h"
#include "../config.h"
#include "../mod_math/vector3.h"

namespace Video
{

/**
 *  Struct for storing video buffer depth information.
 */
typedef struct
{
  int red;      ///< bpp of red component
  int green;    ///< bpp of green component
  int blue;     ///< bpp of blue component
  int alpha;    ///< bpp of alpha component
  int depth;    ///< bpp of depth buffer
  int stencil;  ///< bpp of stencil buffer
} T_VideoBitDepthInfo;


// --- global variables defined in crrc_graphics.cpp -------

extern T_VideoBitDepthInfo vidbits;

extern ssgRoot *scene;

extern int window_xsize;  // Size of window in x direction
extern int window_ysize;  // Size of window in y direction
extern int screen_xsize;  // Size of screen in x direction
extern int screen_ysize;  // Size of screen in y direction

extern float flSmartCam;
extern float flSmartCam_time;
extern float flSmartCam_center_w;
extern float flSmartCam_center_h;
extern float flSmartCam_var_w;
extern float flSmartCam_var_h;
extern float flSloppyCam;

extern CRRCMath::Vector3 looking_pos;

// --- functions defined in crrc_graphics.cpp --------------

unsigned char * read_bwimage(const char *name, int *w, int *h);
unsigned char * read_rgbimage(const char *name,int *w, int *h);
GLuint make_texture(unsigned char *pixel_data, GLint pixel_format, GLint format,
                    GLsizei width, GLsizei height, bool use_mipmaps);

void reshape(int w, int h);
void drawSolidCube(GLfloat size);
void dumpGLStackInfo(FILE* pFile);

ssgContext* getGlobalRenderingContext();

} // end namespace Video::

#endif
