/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2009 Jan Reucker (original author)
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
 * \file global_video.h
 *
 * Global interface for mod_video.
 */

#ifndef GLOBAL_VIDEO_H
#define GLOBAL_VIDEO_H

#include "mod_misc/SimpleXMLTransfer.h"
#include "mod_math/vector3.h"

// We need this header as long as pure OpenGL stuff is exposed
// through this interface
#include "include_gl.h"

namespace Video
{
/// \todo #define's should only be declared once. Right now all
///       global defines from mod_video are defined twice.

#define INVALID_AIRPLANE_VISUALIZATION -1

#if defined(__APPLE__) || defined(MACOSX)
#define DEFAULT_SKYBOX_TEXTURE_OFFSET (0.0009f)
#else
#define DEFAULT_SKYBOX_TEXTURE_OFFSET (0.0f)
#endif

void display();
void read_config(SimpleXMLTransfer* cf);
void initialize_scenegraph();
void adjust_zoom(float field_of_view);
void cleanup();
int  setupScreen(int nX, int nY, int nFullscreen);
void setWindowTitleString();
void drawSolidCube(GLfloat size);
void resize_window(int w, int h);
void shadowVolumeDrawShadows();

/**
 * Convert from FDM to graphics reference and backwards
 */
CRRCMath::Vector3 FDM2Graphics(CRRCMath::Vector3 const& v);
CRRCMath::Vector3 Graphics2FDM(CRRCMath::Vector3 const& v);
  
/**
 * Initialise smart camera infos
 */
void InitSmartCamera();
  
/**
 * calculate looking direction
 */
void UpdateCamera(float flDeltaT);
  
/**
 * Get the size of the current window
 */
void getWindowSize(int& x, int& y);

/**
 * Read the "smart cam" setting from mod_video
 */
float getSmartCam();

/**
 * Write the "smart cam" setting
 * \param flValue  New value for smart cam
 */
void setSmartCam(float flValue);

/**
 * Read the "sloppy cam" setting from mod_video
 */
float getSloppyCam();

/**
 * Write the "sloppy cam" setting
 * \param flValue  New value for sloppy cam
 */
void setSloppyCam(float flValue);

/**
 * Initialize the console
 */
void initConsole();

/**
 * Create a new airplane visualization
 */
long new_visualization( std::string const& model_name,
                        std::string const& texture_path,
                        CRRCMath::Vector3 const& pCG,
                        SimpleXMLTransfer *xml);

/**
 * Deallocate an airplane visualization
 */
void delete_visualization(long id);

/**
 * Update the position of a visualization
 */
void set_position(long id,
                  CRRCMath::Vector3 const &pos,
                  double phi,
                  double theta,
                  double psi);

/**
 * Reset the shadow of a visualization
 */
void reset_shadow(long id);

/**
 * Get the bounding sphere radius of a visualisation
 */
float get_radius(long id);

/**
 * Draw a single airplane visualisation
 * Either the real object or the shadow volume object is drawn
 */
void draw_airplane(long id, double current_time, bool fObject = true);

/**
 * Draw all airplanes for which a visualisation has been created
 * Either the real objects or their shadow volume objects are drawn
 */
void draw_all_airplanes(double current_time, bool fObject = true);

/**
 * Configure the sky visualization
 */
class SkyRenderer;
SkyRenderer *setup_sky(SimpleXMLTransfer *xml);

} // end namespace Video::

#endif // GLOBAL_VIDEO_H
