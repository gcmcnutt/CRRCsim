/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2008 Olivier Bordes (original author)
 * Copyright (C) 2005 Lionel Cailler
 * Copyright (C) 2005, 2009-2010 Jens Wilhelm Wulf
 * Copyright (C) 2005-2009 Jan Reucker
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
 * \file handlerF3A.cpp
 *
 * Purpose: Add  F3A functions to crrcsim. 
 *          F3A is a FAI category which define aerobatics contest
 */
#include "../../i18n.h"
#include "../../include_gl.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include "../../global.h"
#include "../../aircraft.h"
#include "../../crrc_soundserver.h"
#include "../../global_video.h"
#include "../../crrc_system.h"
#include "../../mod_misc/ls_constants.h"
#include "../../mod_misc/SimpleXMLTransfer.h"
#include "../../mod_misc/lib_conversions.h"
#include "../../mod_misc/filesystools.h"
#include "../../mod_landscape/crrc_scenery.h"
#include "../../record.h"
#include "../../robots.h"
#include "../../mod_robots/marker.h"
#include "../../mod_video/fonts.h"
#include "../../mod_video/gloverlay.h"
#include "../../SimStateHandler.h"
#include "handlerF3A.h"
using namespace std;

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/** \brief The default constructor
 *
 *  Creates an F3A game handler
 */
HandlerF3A::HandlerF3A() 
{
  prepareConfigFile(cfgfile);
  //retrieve information from config file
  draw_grid = cfgfile->getInt("game.f3a.draw_grid");
  draw_trajectory = cfgfile->getInt("game.f3a.draw_trajectory");
  draw_indicators = cfgfile->getInt("game.f3a.draw_indicators");
  persistence = cfgfile->getInt("game.f3a.trajectory_persistence");
  tolerance = cfgfile->getInt("game.f3a.attitude_tolerance");

  SimpleXMLTransfer* f3acfg = cfg->getCurLocCfgPtr(cfgfile)->getChild("game.f3a", true);
  grid_size = f3acfg->getInt ("grid_size");
  flight_height = f3acfg->getInt ("flight_height");
  flight_distance = f3acfg->getInt ("flight_distance");
  set_orientation(f3acfg->getInt("orientation") );
  center_grid_position_north = f3acfg->getInt("position_north");
  center_grid_position_east  = f3acfg->getInt("position_east");
  center_grid_position_elev  = Global::scenery->getHeight(
                                 center_grid_position_north,
                                 center_grid_position_east);

  window_xsize = 0;
  window_ysize = 0; 

  // states for OpenGL rendering
  trajectory_rendering_state = new ssgSimpleState();
  trajectory_rendering_state->disable(GL_CULL_FACE);
  trajectory_rendering_state->disable(GL_COLOR_MATERIAL);
  trajectory_rendering_state->disable(GL_TEXTURE_2D);
  trajectory_rendering_state->disable(GL_LIGHTING);
  trajectory_rendering_state->enable(GL_BLEND);
  trajectory_rendering_state->enable(GL_LINE_SMOOTH);
      
  grid_rendering_state = new ssgSimpleState();
  grid_rendering_state->disable(GL_CULL_FACE);
  grid_rendering_state->disable(GL_COLOR_MATERIAL);
  grid_rendering_state->disable(GL_TEXTURE_2D);
  grid_rendering_state->disable(GL_LIGHTING);
  grid_rendering_state->enable(GL_BLEND);
  grid_rendering_state->enable(GL_LINE_SMOOTH);
    
  text_rendering_state = new ssgSimpleState();
  text_rendering_state->disable(GL_CULL_FACE);
  text_rendering_state->disable(GL_COLOR_MATERIAL);
  text_rendering_state->disable(GL_TEXTURE_2D);
  text_rendering_state->disable(GL_LIGHTING);
  text_rendering_state->enable(GL_BLEND);
  text_rendering_state->setMaterial(GL_EMISSION, 0.0, 0.0, 0.0, 0.0);
  text_rendering_state->setMaterial(GL_AMBIENT, 1.0, 1.0, 1.0, 1.0);
  text_rendering_state->setMaterial(GL_DIFFUSE, 1.0, 1.0, 1.0, 1.0);
  text_rendering_state->setMaterial(GL_SPECULAR, 0.0, 0.0, 0.0, 0.1);
  
  reset();
}


/** \brief The destructor.
 *
 *  Deletes an F3A game handler
 */
HandlerF3A::~HandlerF3A()
{
  delete grid_rendering_state;
  delete trajectory_rendering_state;
  delete text_rendering_state;
}


/** \brief Render game-mode-specific details
 *
 *  This method renders graphics objects that are specific to the
 *  F3F game mode, namely the base pylons.
 */
void HandlerF3A::draw()
{
  if (Global::testmode)
    return;
    
  if (draw_grid)
  {
    grid_rendering_state->apply();
    draw_f3a_grid();
  }
  if (draw_trajectory)
  {
    trajectory_rendering_state->apply();
    draw_f3a_trajectory();
  }  
}


/** \brief Print the game-mode-specific text overlay
 *
 *  This method renders the text overlay for the F3A mode.
 *
 *  \todo This method should use PLIB instead of GLUT for font rendering.
 *
 *  \param  ww  current OpenGL window width
 *  \param  hh  current OpenGL window height
 */
void HandlerF3A::display_infos(GLfloat ww, GLfloat hh)
{   
  int y_offset;
  int lineheight = 35;
  char astring[256];
  
  window_xsize = ww;
  window_ysize = hh;

  if (draw_indicators)
  {
    // draw indicators overlay
    GlOverlay::setupRenderingState(window_xsize, window_ysize);
    draw_f3a_indicators(window_xsize, window_ysize);
    GlOverlay::restoreRenderingState();
  }

  startTextRendering();
  text_rendering_state->apply();
  
  if (ww <= 800)
  {
    if (Video::textureFont)
    {
      fontRenderer.setFont(Video::textureFont);
      fontRenderer.setPointSize(16);
    }
    else fontRenderer.setFont(fntGetBitmapFont(FNT_BITMAP_HELVETICA_18));
    y_offset = 45;
    lineheight = 27;
  }
  else
  {
    if (Video::textureFont)
    {
      fontRenderer.setFont(Video::textureFont);
      fontRenderer.setPointSize(22);
    }
    else fontRenderer.setFont(fntGetBitmapFont(FNT_BITMAP_TIMES_ROMAN_24));
    y_offset = 55;
    lineheight = 36;
  }

  // debug string
  sprintf(astring, "- F3A -");
  output(window_xsize/2 - textLength(astring)/2,
         window_ysize - y_offset - 0*lineheight, 
         astring);

  finishTextRendering();
}


/** \brief Switch to a text rendering state/projection
 *
 *  Sets up the OpenGL projection matrix for 2D text rendering.
 */
void HandlerF3A::startTextRendering() const
{
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();

  glLoadIdentity();
  gluOrtho2D(0, window_xsize, 0, window_ysize);
}


/** \brief Switch back to 3D rendering
 *
 *  Revert the changes done in startTextRendering()
 */
void HandlerF3A::finishTextRendering() const
{
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
}


/** \brief Render a text string
 *
 *  This method renders a string at a given position using the
 *  specified font.
 *
 *  \todo This method should use PLIB instead of GLUT for font rendering.
 *
 *  \param  x     Horizontal start of text
 *  \param  y     Vertical start of text
 *  \param  text  the string to be displayed
 */
void HandlerF3A::output(GLfloat x, GLfloat y, const char *text)
{
  fontRenderer.begin();
  glColor4f(1, 0.3, 0.3, 0.7);
  fontRenderer.start2f(x, y);
  fontRenderer.puts(text);
  fontRenderer.end();
}


/** \brief Calculate the length of a rendered text string
 *
 *  This method calculates the length of a text string rendered
 *  in the current font.
 *
 *  \param    text    pointer to the text to be rendered
 *  \return   width of the rendered text string
 */
GLfloat HandlerF3A::textLength(const char *text) const
{
  float left, right;
  
  fontRenderer.getFont()->getBBox(text, 
                                  fontRenderer.getPointSize(),
                                  fontRenderer.getSlant(),
                                  &left, &right,
                                  NULL, NULL);
  return (right - left);
}


/** \brief Reset the game handler.
 *
 *  Everything will be reset to be ready for a new run.
 */
void HandlerF3A::reset()
{
  // reset trajectory storage
  traj_t0 = 0;
  traj_first = MAX_TRAJ - 1;
  traj_last = 0;
  
  // reset trajectory slope angle
  THETA_cg = 0.;
}


/** \brief Cyclic game-handler function for F3A
 *
 *  Check if the model cross pylons A or pylons B.
 *  Increase BASE counter.
 */
void HandlerF3A::update(float a, float b, float c, FlightRecorder* recorder, Robots* robots)
{
  if (Global::testmode)
    return;

  // save previous cg coords
  float xx_cg_old = XX_cg_grd;
  float yy_cg_old = YY_cg_grd;
  float zz_cg_old = ZZ_cg_grd;
  
  //convert coords on F3A coords
  float a0 = a - center_grid_position_north;
  float b0 = b - center_grid_position_east;
  float c0 = c - center_grid_position_elev;
  XX_cg_grd =  - b0 * cos_dir + a0 * sin_dir;
  YY_cg_grd =  + a0 * cos_dir + b0 * sin_dir;
  ZZ_cg_grd =  c0;
  
  // compute cg trajectory slope angle
  THETA_cg = 0.0;
  if (traj_t0)
  {
    float dx = XX_cg_grd - xx_cg_old;
    float dy = YY_cg_grd - yy_cg_old;
    float dz = ZZ_cg_grd - zz_cg_old;
    float dl = sqrt(dx*dx + dy*dy + dz*dz);
    if (dl > 0.1)
    {
      float dd = sqrt(dx*dx + dy*dy);
      THETA_cg = atan2(dz,dd);
    }
  }

  // advance buffer index, avoid overwriting
  if (++traj_first == MAX_TRAJ)
    traj_first = 0;
  if (traj_t0 && (traj_first == traj_last))
  {
    if (++traj_last == MAX_TRAJ)
      traj_last = 0;
  }

  // save trajectory in circular buffer
  traj_x[traj_first] = a;
  traj_y[traj_first] = b;
  traj_z[traj_first] = c;
  traj_t[traj_first] = Global::Simulation->getSimulationTimeSinceReset();
  traj_CL[traj_first] = Global::aircraft->getFDM()->getFlightCL();
  if (!traj_t0)
    traj_t0 = traj_t[traj_first];
 
  // identify last time sample
  int t_last = traj_t[traj_first] - persistence*1000;
  if (t_last < traj_t0)
    t_last = traj_t0;
  while (traj_t[traj_last] < t_last)
  {
    if (++traj_last == MAX_TRAJ)
      traj_last = 0;
  }
}

/** \brief Prepare the config file
 *
 * This function checks if the config file contains all tags needed
 * to store the F3A configuration. If a tag is missing, it is
 * created and filled with a sensible default value.
 *
 * \param cfgfile   Pointer to the config file
 */
void HandlerF3A::prepareConfigFile(SimpleXMLTransfer *cfgfile)
{
  int grid_size, flight_height, flight_distance, orientation, position_north, position_east;

  //general F3A-options
  cfgfile->makeSureAttributeExists("game.f3a.enabled", "0");
  cfgfile->makeSureAttributeExists("game.f3a.draw_grid", "0");
  cfgfile->makeSureAttributeExists("game.f3a.draw_trajectory", "0");
  cfgfile->makeSureAttributeExists("game.f3a.draw_indicators", "0");
  cfgfile->makeSureAttributeExists("game.f3a.trajectory_persistence", "30");
  cfgfile->makeSureAttributeExists("game.f3a.attitude_tolerance", "2");
  
  //location specifics parameters
  SimpleXMLTransfer *xml_scenery = Global::scenery->getXMLsection("F3A");
  if (!xml_scenery) xml_scenery = Global::scenery->getXMLsection("f3a");
  if (xml_scenery)
  {
    //retrieve information from scenery file description
    grid_size = xml_scenery->attributeAsInt("grid_size", 100);
    flight_height = xml_scenery->attributeAsInt("flight_height", 100);
    flight_distance = xml_scenery->attributeAsInt("flight_distance", 500);
    orientation = xml_scenery->attributeAsInt("orientation", 0);
    position_north = xml_scenery->attributeAsInt("position_north", 0);
    position_east = xml_scenery->attributeAsInt("position_east", 0);
  }
  else
  {
    //default value
    grid_size = 100;
    flight_height = 100;
    flight_distance = 500;
    orientation = 0;
    position_north = 0;
    position_east = 0;
  }
  //put on location section on configfile 
  SimpleXMLTransfer* f3acfg = cfg->getCurLocCfgPtr(cfgfile)->getChild("game.f3a", true);
  char buf[100];
  sprintf(buf,"%d",grid_size);
  f3acfg->makeSureAttributeExists("grid_size", buf);
  sprintf(buf,"%d",flight_height);
  f3acfg->makeSureAttributeExists("flight_height", buf);
  sprintf(buf,"%d",flight_distance);
  f3acfg->makeSureAttributeExists("flight_distance", buf);
  sprintf(buf,"%d",orientation);
  f3acfg->makeSureAttributeExists("orientation", buf);
  sprintf(buf,"%d",position_north);
  f3acfg->makeSureAttributeExists("position_north",buf);
  sprintf(buf,"%d",position_east);
  f3acfg->makeSureAttributeExists("position_east",buf);
}


SimpleXMLTransfer* HandlerF3A::GetRecordHeader()
{
  SimpleXMLTransfer* header = new SimpleXMLTransfer();
  header->setAttribute("mode", "F3A");
  return(header);
}


void HandlerF3A::draw_f3a_grid() const
{
  GLfloat htan = tan(H_ANGLE*M_PI/180.);
  GLfloat vtan = tan(V_ANGLE*M_PI/180.);
  GLfloat xl, xr, y, zb, zt;
  
  // grid plane dimensions, relative to base center
  // y positive to the front, x positive to the left
  y  = flight_distance;
  xr = y*htan;
  xl = - xr;
  zb = flight_height;
  zt = y*vtan;
  
  draw_f3a_grid_plane(xl, xr, y, zb, zt);
}


void HandlerF3A::draw_f3a_grid_plane(GLfloat xl, GLfloat xr, GLfloat y, GLfloat zb, GLfloat zt) const
{
  GLfloat ln, le, cn, ce, rn, re, z1, z2;
  GLfloat x, z;
  GLfloat alpha0 = 0.5;
  GLfloat color_alpha[] = {1,0,0, alpha0};
  
  // grid plane world coords
  ln = center_grid_position_north + y * cos_dir - xl * sin_dir;
  cn = center_grid_position_north + y * cos_dir;
  rn = center_grid_position_north + y * cos_dir - xr * sin_dir;
  le = center_grid_position_east  + y * sin_dir + xl * cos_dir;
  ce = center_grid_position_east  + y * sin_dir;
  re = center_grid_position_east  + y * sin_dir + xr * cos_dir;
  z1 = center_grid_position_elev  + zb;
  z2 = center_grid_position_elev  + zt;
  
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glColor4fv(color_alpha);

  // draw grid bottom and center line
  glLineWidth(4.0);
  glBegin(GL_LINES);
  glVertex3f(le, z1, -ln);
  glVertex3f(re, z1, -rn);
  glEnd();
  
  glBegin(GL_LINES);
  glVertex3f(ce, z1, -cn);
  glVertex3f(ce, z2, -cn);
  glEnd();

  // draw grid perimeter
  color_alpha[3] = alpha0*0.2;
  glColor4fv(color_alpha);
  glLineWidth(4.0);
  glBegin(GL_LINE_STRIP);
  glVertex3f(le, z1, -ln);
  glVertex3f(le, z2, -ln);
  glVertex3f(re, z2, -rn);
  glVertex3f(re, z1, -rn);
  glEnd();

  // draw vertical grid lines
  glLineWidth(1.0);
  x = 0.0 + grid_size;
  while (x < xr)
  {
    color_alpha[3] = alpha0*(0.8*(1. - x/xr) + 0.2);
    glColor4fv(color_alpha);
    glBegin(GL_LINES);
    glVertex3f(ce - x*cos_dir, z1, -(cn + x*sin_dir));
    glVertex3f(ce - x*cos_dir, z2, -(cn + x*sin_dir));
    glVertex3f(ce + x*cos_dir, z1, -(cn - x*sin_dir));
    glVertex3f(ce + x*cos_dir, z2, -(cn - x*sin_dir));
    glEnd();
  
    x += grid_size;
  }

  // draw horizontal grid lines
  glLineWidth(1.0);
  z = z1 + grid_size;
  while (z < z2)
  {
    color_alpha[3] = alpha0*(0.8*(1. - (z - z1)/(z2 - z1)) + 0.2);
    glColor4fv(color_alpha);
    glBegin(GL_LINES);
    glVertex3f(le, z, -ln);
    glVertex3f(re, z, -rn);
    glEnd();
  
    z += grid_size;
  }
  
  glLineWidth(1.0);
}


void HandlerF3A::draw_f3a_trajectory() const
{ 
  GLfloat alpha0 = 0.4;
  GLfloat a, b, x, y, north, east;
  GLfloat color1_alpha[] = {0,0,0, alpha0};
  GLfloat color2_alpha[] = {1,0,0, alpha0};

  if (traj_t0)
  {
    int t_blur = traj_t[traj_last] + 1000;

    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glLineStipple(2, 0x0F0F); // dashed line pattern

    // draw 3D trajectory line
    int segment = 0;
    int i = traj_last;
    int n = traj_first;
    if (++n == MAX_TRAJ)
      n = 0;
    
    glLineWidth(3.0);
    while (i != n)
    {
      if (traj_t[i] < t_blur)
        color1_alpha[3] = alpha0*(1. - (t_blur - traj_t[i])/1000.);
      else
        color1_alpha[3] = alpha0;
        
      if (!segment)
      {
        if (traj_CL[i] >= 0.0)
        {
          segment = 1;
          glDisable(GL_LINE_STIPPLE);
          glBegin(GL_LINE_STRIP);
        }
        else
        {
          segment = -1;
          glEnable(GL_LINE_STIPPLE);
          glBegin(GL_LINE_STRIP);
        }
      }
      glColor4fv(color1_alpha);
      glVertex3f(traj_y[i], traj_z[i], -traj_x[i]);
      
      if (segment*traj_CL[i] > 0.0)
      {
        if (++i == MAX_TRAJ)
          i = 0;
      }
      else
      {
        glEnd();
        segment = 0;
      }
    }
    glEnd();
    glDisable(GL_LINE_STIPPLE);
    
    // draw 2D trajectory line on grid plane
    if (draw_grid)
    {
      int segment = 0;
      int i = traj_last;
      int n = traj_first;
      if (++n == MAX_TRAJ)
        n = 0;
      
      glLineWidth(3.0);
      while (i != n)
      {
        //convert world coords to F3A grid coords
        a = traj_x[i] - center_grid_position_north;
        b = traj_y[i] - center_grid_position_east;
        x = b * cos_dir - a * sin_dir;
        y = a * cos_dir + b * sin_dir;
        // project to grid plane
        y = flight_distance;
        // convert back to world coords
        north = center_grid_position_north + y * cos_dir - x * sin_dir;
        east  = center_grid_position_east  + y * sin_dir + x * cos_dir;
        
        if (traj_t[i] < t_blur)
          color2_alpha[3] = alpha0*(1. - (t_blur - traj_t[i])/1000.);
        else
          color2_alpha[3] = alpha0;
          
        if (!segment)
        {
          if (traj_CL[i] >= 0.0)
          {
            segment = 1;
            glDisable(GL_LINE_STIPPLE);
            glBegin(GL_LINE_STRIP);
          }
          else
          {
            segment = -1;
            glEnable(GL_LINE_STIPPLE);
            glBegin(GL_LINE_STRIP);
          }
        }
        glColor4fv(color2_alpha);
        glVertex3f(east, traj_z[i], -north);
        
        if (segment*traj_CL[i] > 0.0)
        {
          if (++i == MAX_TRAJ)
            i = 0;
        }
        else
        {
          glEnd();
          segment = 0;
        }
      }
      glEnd();
      glDisable(GL_LINE_STIPPLE);
    }
    
    glLineWidth(1.0);
  }
}


void HandlerF3A::draw_f3a_indicators(int window_xsize, int window_ysize) const
{ 
  // pitch from aircraft attitude
  //GLfloat pitch = Global::aircraft->getFDM()->getTheta() * SG_RADIANS_TO_DEGREES;
  //draw_f3a_horizon(window_xsize, window_ysize, 0, pitch);

  // pitch from cg trajectory slope
  GLfloat pitch = THETA_cg * SG_RADIANS_TO_DEGREES;
  draw_f3a_horizon(window_xsize, window_ysize, 0, pitch);

  GLfloat roll = Global::aircraft->getFDM()->getPhi() * SG_RADIANS_TO_DEGREES;
  draw_f3a_horizon(window_xsize, window_ysize, 1, -roll);
}


void HandlerF3A::draw_f3a_horizon(int window_xsize, int window_ysize, int position, GLfloat angle) const
{ 
  int basex = 1.5*(window_ysize >> 5);
  int basey = 2.0*(window_ysize >> 5);
  int r = window_ysize >> 4;
  int gap = r >> 3;
  int tic = 0.5*gap;
  int w = 2*(r + gap);
  int h = 2*gap;
  int i;
  
  GLfloat alpha = 0.6;
  GLfloat white[] = {1,1,1};
  GLfloat black[] = {0,0,0};
  GLfloat red[]   = {1,0,0};
  GLfloat white_alpha[] = {1,1,1, alpha};
  GLfloat black_alpha[] = {0,0,0, alpha};
  GLfloat ground_alpha[] = {.375,.275,0, alpha};
  GLfloat sky_alpha[] = {0,.75,1, alpha};

  GLUquadricObj *quadric;
  quadric = gluNewQuadric();
  
  // draw pitch or roll indicator
  
  basey += position*w; // stack indicators on top of previous

  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glPushMatrix();
  glTranslatef(basex + w/2, basey + w/2, 0);
  
  // horizon
  glColor4fv(black);
  gluDisk(quadric, r, r+2, 32, 1);
  //glRecti(-w/2, -w/2, w/2, w/2);
  glColor4fv(ground_alpha);
  gluPartialDisk(quadric, 0, r, 32, 1, 90 + angle, 180);
  glColor4fv(sky_alpha);
  gluPartialDisk(quadric, 0, r, 32, 1, 270 + angle, 180);
  
  // marks
  glColor3fv(white);
  for (i = 0; i < 24; i++)
  {
    if (i % 3)
    {
      glLineWidth(1.0);
      glBegin(GL_LINES);
      glVertex2i(      r, 0);
      glVertex2i(r - tic, 0);
      glEnd();
    }
    else
    {
      glLineWidth(2.0);
      glBegin(GL_LINES);
      glVertex2i(      r, 0);
      glVertex2i(r - gap, 0);
      glEnd();
    }
    glRotatef(15, 0, 0, 1);
  }
  
  // airplane silouette
  if (fabs(angle - 45.0*floor((angle + 0.5)/45.0)) <= tolerance)
    glColor3fv(red);
  else
    glColor3fv(black);
  glLineWidth(2.0);
  glBegin(GL_LINES);
  glVertex2i(-(r - gap), 0);
  glVertex2i(   r - gap, 0);
  glEnd();
  if (position == 0)
  {
    // pitch indicator 
    glBegin(GL_TRIANGLES);
    glVertex2i(    r - gap,  0);
    glVertex2i(r - gap - h,  h/2);
    glVertex2i(r - gap - h, -h/2);
    glEnd();
    
    glColor4fv(black_alpha);
    gluPartialDisk(quadric, 0, h, 16, 1,   0, 90);
    gluPartialDisk(quadric, 0, h, 16, 1, 180, 90);
    glColor4fv(white_alpha);
    gluPartialDisk(quadric, 0, h, 16, 1,  90, 90);
    gluPartialDisk(quadric, 0, h, 16, 1, 270, 90);    
  }
  else
  {
    // roll indicator
    glBegin(GL_TRIANGLES);
    glVertex2i(-h/2, 0);
    glVertex2i( 0, h);
    glVertex2i( h/2, 0);
    glEnd();
  }
  glLineWidth(1.0);
  glPopMatrix();
  
  gluDeleteQuadric(quadric);
}
