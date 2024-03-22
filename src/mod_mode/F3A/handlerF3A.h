/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2008 Olivier Bordes (original author)
 * Copyright (C) 2005, 2006 Jan Reucker
 * Copyright (C) 2010 Jens Wilhelm Wulf
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
 * \file handlerF3A.h
 *
 * Purpose:  This file add the F3A function to crrcsim
 */

#ifndef __HANDLERF3A_H
#define __HANDLERF3A_H


#include <string>
#include <plib/ssg.h>   // for ssgSimpleState
#include <plib/fnt.h>   // for fntRenderer
#include "../../config.h"
#include "../T_GameHandler.h"

#define MAX_TRAJ 20000 // trajectory samples, up to approx 30s

#define H_ANGLE 60.0
#define V_ANGLE 60.0

class HandlerF3A : public  T_GameHandler
{
  public:
    HandlerF3A();
    virtual ~HandlerF3A();
  
    /**
     * Disable this game mode (because users has enabled another one)
     */
    void disable() 
    {
      cfgfile->setAttributeOverwrite("game.f3a.enabled", 0);
    };
    
    /**
     * calculate aircraft F3A position
     */
    void update(float a, float b, float c, FlightRecorder* recorder, Robots* robots);

    /**
     *  draw F3A grid and display flight informations
     */
    void draw();
  
    /**
     *  draw the info text
     */
    void display_infos(GLfloat w, GLfloat h);

    /** restart game */
    void reset();
    
    /** define game type */
    std::string gameType() const {return std::string("F3A");};

    /** public set method for draw_indicators */
    inline void set_draw_indicators(int aValue) {draw_indicators = aValue;};
    
    /** public set method for draw_trajectory */
    inline void set_draw_trajectory(int aValue) {draw_trajectory = aValue;};

    /** public set method for draw_grid */
    inline void set_draw_grid(int aValue) {draw_grid = aValue;};
    
    /** public set method for angle tolerance*/
    inline void set_tolerance(int aValue) {tolerance = aValue;};
    
    /** public set method for persistence */
    inline void set_persistence(int aValue) {persistence = aValue;};
    
    /** public set method for grid_size */
    inline void set_grid_size(int aValue) {grid_size = aValue;};
    
    /** public set method for flight_height */
    inline void set_flight_height(int aValue) {flight_height = aValue;};
    
    /** public set method for flight_distance */
    inline void set_flight_distance(int aValue) {flight_distance = aValue;};
    
    /** public set method for orientation */
    inline void set_orientation(int aValue)
    {
      orientation = aValue;
      dir = orientation/180.0*M_PI;
      cos_dir = cos(dir);
      sin_dir = sin(dir);
    };
    
    /** public set methods for position */
    inline void set_position_n(int aValue) {center_grid_position_north = aValue;};
    inline void set_position_e(int aValue) {center_grid_position_east = aValue;};
    
    static void prepareConfigFile(SimpleXMLTransfer *cfgfile);
  
    /**
     * The game mode returns a header which is written at the beginning of
     * a recorded file.
     */
    SimpleXMLTransfer* GetRecordHeader();

    /**
     * The game mode returns the center and half the angular extent of the game scene, in FDM coords
     */
    int GetSmartCameraPar(float& phi_w0, float& phi_h0, float& range_w, float& range_h)
    {
      phi_w0  = dir;
      phi_h0  = -30. * M_PI/180.; // Z is positive downward, so angle is negative
      range_w = 0.5 * 120. * M_PI/180.;
      range_h = 0.5 *  60. * M_PI/180.;
      
      return 1;
    };
    
  private:
    //drawing and game informations
    void output(GLfloat x, GLfloat y, const char *text);
    void draw_f3a_grid() const;
    void draw_f3a_grid_plane(GLfloat xl, GLfloat xr, GLfloat y, GLfloat zb, GLfloat zt) const;
    void draw_f3a_trajectory() const;
    void draw_f3a_indicators(int wx, int wy) const;
    void draw_f3a_horizon(int wx, int wy, int pos, GLfloat angle) const;
    void startTextRendering() const;
    void finishTextRendering() const;
    GLfloat textLength(const char *text) const;

    int traj_t0;
    int traj_first;
    int traj_last;
    int draw_indicators;
    int draw_trajectory;
    int draw_grid;
    int tolerance; // (degrees) tolerance for pitch/roll angle display
    int persistence;
    int grid_size;
    int flight_height;
    int flight_distance;
    int orientation; //(degrees) bearing angle looking at the grid center
    float dir; // orientation angle, radians
    float cos_dir, sin_dir;   
    float center_grid_position_north;
    float center_grid_position_east;
    float center_grid_position_elev;

    float XX_cg_grd;
    float YY_cg_grd;
    float ZZ_cg_grd;
    float THETA_cg;
    
    float traj_x[MAX_TRAJ];
    float traj_y[MAX_TRAJ];
    float traj_z[MAX_TRAJ];
    float traj_t[MAX_TRAJ];
    float traj_CL[MAX_TRAJ];

    GLfloat window_xsize;
    GLfloat window_ysize;
    
    ssgSimpleState *grid_rendering_state;    ///< state for rendering grid
    ssgSimpleState *trajectory_rendering_state;    ///< state for rendering trajectory
    ssgSimpleState *text_rendering_state;     ///< state for rendering text

    fntRenderer   fontRenderer;
};

#endif
