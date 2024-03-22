/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2006, 2007 Jan Reucker (original author)
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
  

// puaImageFrame.cpp - a frame housing a texture (image).

#include "puaImageFrame.h"


/**
 * Create a puaImageFrame
 */
puaImageFrame::puaImageFrame (int minx, int miny, int maxx, int maxy, ssgTexture *image)
: puFrame(minx, miny, maxx, maxy)
{
  setTexture(image);
  this->setRenderCallback (puaImageFrameRenderCallback, this);
}


/**
 * Change texture (image) drawn inside the frame
 */
void puaImageFrame::puaImageFrameRenderCallback(puObject *obj, int x0, int y0, void *tmp) 
{
  ssgTexture *image = ((puaImageFrame*)obj)->_image;
  if (image)
  {
    int x, y, dx, dy;
    obj->getPosition(&x, &y);
    obj->getSize(&dx, &dy);
    x += x0;
    y += y0;
    glEnable(GL_TEXTURE_2D);
    GLuint tex = image->getHandle();
    glBindTexture(GL_TEXTURE_2D, tex);
    glBegin(GL_POLYGON);
        glTexCoord2f(0, 0);glVertex2f(x,    y);
        glTexCoord2f(1, 0);glVertex2f(x+dx, y);
        glTexCoord2f(1, 1);glVertex2f(x+dx, y+dy);
        glTexCoord2f(0, 1);glVertex2f(x,    y+dy);
    glEnd();
    glDisable(GL_TEXTURE_2D);
  }
}


// end of puaImageFrame.cpp
