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
 * \file handlerF3F.h
 *
 * Purpose:  This file add the F3F function to crrcsim
 * History:  See crrcsim-f3f version 1.5. See also web site obor.free.fr 
 */

#ifndef __HANDLERF3F_H
#define __HANDLERF3F_H


#include <string>
#include <plib/ssg.h>   // for ssgSimpleState
#include <plib/fnt.h>   // for fntRenderer
#include "../../config.h"
#include "../T_GameHandler.h"
#define MAX_LAPS 10  
#define PYLON_LEFT 1
#define PYLON_RIGHT 2

class HandlerF3F : public  T_GameHandler
{
  public:
    HandlerF3F();
    virtual ~HandlerF3F();
  
    /**
     * Disable this game mode (because users has enabled another one)
     */
    void disable() 
    {
      cfgfile->setAttributeOverwrite("game.f3f.enabled", 0);
    };

    /**
     * calculate aircraft F3F position
     */
    void update(float a, float b, float c, FlightRecorder* recorder, Robots* robots);

    /**
     *  draw F3F bases and display flight informations
     */
    void draw();
  
    /**
     *  draw the info text
     */
    void display_infos(GLfloat w, GLfloat h);

    /** restart game */
    void reset();
    
    /** define game type */
    std::string gameType() const {return std::string("F3F");};

    /** public set method for extend_base */
    inline void set_extend_bases(int aValue) {extend_base = aValue;};
    
    /** public set method for security_line */
    inline void set_security_line(int aValue) {security_line = aValue;};
    
    /** public set method for orientation */
    inline void set_orientation(int aValue)
    {
      orientation_base = aValue;
      dir = orientation_base/180.0*M_PI;
      cos_dir = cos(dir);
      sin_dir = sin(dir);
    };
    
    /** public set methods for position */
    inline void set_position_n(int aValue) {center_base_position_north = aValue;};
    inline void set_position_e(int aValue) {center_base_position_east = aValue;};
    
    /** public set method for plan_limit */
    inline void set_plan_limit(int aValue)
    {
      plan_limit = aValue/2;
      printf ("F3F: Setting plan limit to:%d\n",plan_limit);
    };
    
    /** public set method for start_on_left */
    inline void set_start_left(int aValue) {start_on_left = aValue;};
    
    /** public set method for the sound directory */
    inline void set_sound_dir(std::string aDir) 
    {
      f3f_sound_dir = aDir;
      use_beep = (f3f_sound_dir == "beep");
      if (use_beep)
        printf("F3F: Using console beep instead of wav sounds\n");
      else
        printf ("F3F: Setting sound dir to: %s\n", f3f_sound_dir.c_str());
    };
    
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
      phi_h0  = 0. * M_PI/180.; // Z is positive downward, so angle is negative
      float distance = fabs(YY_cg_rwy) + 30; // flight distance from player, in ft
      range_w = atan(plan_limit/distance);
      range_h = 0.5 * 30. * M_PI/180.;
      
      return 1;
    };

  private:
    //drawing and game informations
    void output(GLfloat x, GLfloat y, const char *text);
    void draw_f3f_base(GLfloat fx, GLfloat fy, GLfloat fh) const;
    void draw_f3f_base_plane(GLfloat fx, GLfloat fy, GLfloat fh, GLfloat dx, GLfloat dy, int b) const;
    void draw_f3f_base_xy(GLfloat x, GLfloat y) const;
    void draw_f3f_base_plane_xy(GLfloat x, GLfloat y, int b) const;
    void startTextRendering() const;
    void finishTextRendering() const;
    GLfloat textLength(const char *text) const;

    //sound
    int play(std::string soundName);
    int play(std::string soundName, int base);
    int start_sound_id;
    //time
    void update_time ();
    void run_start();
    void end_run(FlightRecorder* recorder);
    void update_turntime ();
    void update_lost_meters ();

    char f3f_score [1024];
    char f3f_time [256];
    char f3f_turntime[256];
    char f3f_meters [256];
    char f3f_result [512];
    char f3f_penalty [256];

    int plan_limit;//Distance between bases
    int orientation_base; //(degres) angle of front of bases. (normaly egal wind direction)
    float dir; // orientation angle, radians
    float cos_dir, sin_dir;   
    int center_base_position_north;
    int center_base_position_east;
    int extend_base;
    int pausetime ;
    int currtime ;
    int runstarttime ;
    int runtime ;
    int runlaststart ;
    int runcurrentstart;
    int runcurrentstop;
    int runcurrentturn;
    int runturntime ;
    int resettime ;
    std::string start_time;
    std::string end_time;

    int run_will_start ;
    int run_started ;
    int run_completed ;
    int start_on_left;
    int x1_toggle ;
    int x2_toggle ;

    int base_count ;
    int next_base_to_cross ;
    int in_penalty ;
    int penality_count;
    int start_from_ground ;

    int elapsed_time[MAX_LAPS];
    int lostm[MAX_LAPS];
    int turntime[MAX_LAPS];

    int lost_meters_rel;
    int lost_meters_abs;

    float XX_cg_rwy;
    float YY_cg_rwy;
    float ZZ_cg_rwy;
    int repeat_mode;
    int security_line;
   
    GLfloat window_xsize;
    GLfloat window_ysize;
    
    ssgSimpleState *pylon_rendering_state;    ///< state for rendering pylons
    ssgSimpleState *plane_rendering_state;    ///< state for rendering base planes
    ssgSimpleState *text_rendering_state;     ///< state for rendering text

    fntRenderer   fontRenderer;

    /* sounds files */
    std::string f3f_sound_dir;      ///< the sound directory
    std::string f3f_soundStart;     ///< the pre-start countdown
    std::string f3f_soundPenalty;   ///< crossed the penalty line
    std::string f3f_soundEntry;     ///< entered the start zone
    std::string f3f_soundFirst;     ///< passed the first base
    std::string f3f_soundBase;      ///< passed base 2-9
    std::string f3f_soundLast;      ///< passed the last base
    
    bool use_beep;    ///< use console beep instead of real sound
    bool F3FMarkerSend; ///< to make sure this marker is only send once after reset
};

#define MAX_START_TIME  30000

#ifdef WIN32
#define SYSTIMETOINT(st)  (st.wMilliseconds + 1000*st.wSecond + 60000*st.wMinute + 3600000*st.wHour )// + 79800000*st.wDay)
#define MAX_START_TIME  30000
#endif
#endif

#define UPDATE_SCORE sprintf ((char*) f3f_score, _("- Lap: %d -- Lap time: %d.%d s -"),base_count, (runcurrentstop - runlaststart)/1000, ((runcurrentstop - runlaststart)%1000)/10)
#define UPDATE_COMPLETE sprintf ((char*) f3f_score,_("- Time: %d.%d s - Penalties: %d - Lost: %d m - Turns: %d.%d s"), (runtime)/1000, (runtime%1000)/10, penality_count, lost_meters_abs, runturntime/1000, runturntime%1000/10)
#define UPDATE_PENALTY sprintf ((char*) f3f_penalty,_("- Penalty: %d -"), penality_count)
#define UPDATE_METERS sprintf ((char*) f3f_meters,_("- Lost: %d m -"), lost_meters_rel)
#define UPDATE_TURN sprintf ((char*) f3f_turntime,_("- Turn: %d.%d s -"), (runcurrentturn - runcurrentstop)/1000, ((runcurrentturn - runcurrentstop)%1000)/10)
#define UPDATE_RESULT sprintf ((char*) f3f_result,"-  %2.2d    %2.2d.%-2.2d %-2s   %2.2d.%-2.2d %-4s   %2.2d %-3s  -",i, itime/1000, (itime%1000)/10, "s", iturn/1000, (iturn%1000)/10, "s", ilost, "m");
#define UPDATE_TIME sprintf ((char*) f3f_time,_("- Time: %d.%d s -"),(runtime)/1000,((runtime)%1000)/10);
//#define UPDATE_MODE sprintf ((char*) f3f_mode,"<%s>", f3f_mode);
#define UPDATE_START {\
  if (start_on_left == TRUE)\
  sprintf ((char*) f3f_score,_(">>> Start on the left pylon, still %d.%d s <<<"), (MAX_START_TIME - runtime)/1000, abs((MAX_START_TIME - runtime)%1000)/10);\
  else \
  sprintf ((char*) f3f_score,_(">>> Start on the right pylon, still %d.%d s <<<"),(MAX_START_TIME - runtime)/1000, abs((MAX_START_TIME - runtime)%1000)/10);\
}

#define PLANLIMIT 328


