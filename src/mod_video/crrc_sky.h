/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 *   Copyright (C) 2004, 2005, 2006, 2007, 2008 Jan Reucker (original author)
 *   Copyright (C) 2005, 2008 Jens Wilhelm Wulf
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

/** \file crrc_sky.h
 *
 *  Declaration of a set of sky rendering classes.
 */
#ifndef _CRRC_SKY_H
#define _CRRC_SKY_H

#include <plib/sg.h>
#include <plib/ssg.h>
#include <plib/ssgaSky.h>
#include <plib/ssgAux.h>
#include "../mod_misc/SimpleXMLTransfer.h"

#include <string>

namespace Video
{

#if defined(__APPLE__) || defined(MACOSX)
#define DEFAULT_SKYBOX_TEXTURE_OFFSET (0.0009f)
#else
#define DEFAULT_SKYBOX_TEXTURE_OFFSET (0.0f)
#endif


void draw_sky(sgVec3 *campos, double dt);
void cleanup_sky();


/** \brief A base class for sky rendering classes.
 *
 *  This base class defines the common interface for
 *  all sky rendering classes. To create a class with this
 *  interface, at least the preDraw() method must
 *  be implemented.
 */
class SkyRenderer
{
  public:
    /// The constructor
    SkyRenderer(SimpleXMLTransfer *xml);
  
    /// The destructor
    virtual ~SkyRenderer() {};
  
    /// Update the camera position and cloud movement
    virtual void update(sgVec3 *campos, double dt) {};
    
    /// Hook for the first drawing stage
    virtual void preDraw() = 0;
  
    /// Hook for the second drawing stage
    virtual void postDraw(float altitude) {};
    
    /// get sky-related lighting parameters for scene rendering
    inline void getAmbientColor(sgVec4 clr) {sgCopyVec4(clr, vSkyAmbientColor);};
    inline void getSunPosition(sgVec3 pos) {sgCopyVec3(pos, vSkySunPosition);};
    inline void getSunColor(sgVec4 clr) {sgCopyVec4(clr, vSkySunColor);};
    inline float getFogDensity() {return flSkyFogDensity;};
    inline void getFogColor(sgVec4 clr) {sgCopyVec4(clr,vSkyFogColor);};

  protected:
    /// ambient light color (and intensity), used for scene rendering
    sgVec4 vSkyAmbientColor;
    /// sun (i.e. light) position
    sgVec3 vSkySunPosition;
    /// sun color (and intensity), used for scene rendering
    sgVec4 vSkySunColor;
    /// fog desity, used for scene rendering
    float flSkyFogDensity;
    /// fog color, used for scene rendering
    sgVec4 vSkyFogColor;
};


/* ******************************************************** */

/** \brief A skybox class for use with PLIB's SSG.
 *
 *  This class creates a cubic skybox.
 */
class SkyBox : public SkyRenderer
{
  private:
    ssgRoot       *skyroot;
    ssgTransform  *skyboxtrans;
    float         boxsize;
  
  public:
    /// Create a skybox
    SkyBox(SimpleXMLTransfer *xml, const char **textures, float size = 20.0, float texoffset = 0.0);
  
    /// Delete a skybox
    ~SkyBox();
  
    /// Update the camera position and cloud movement
    void update(sgVec3 *campos, double dt);
  
    /// Draw the skybox
    void preDraw();
};


/* ******************************************************** */

/** \brief A sky dome class based on PLIB's ssgaSky.
 *
 *  This class renders a sky dome using the ssgaSky class.
 */
#if 0
class SkyDome : public SkyRenderer
{
  private:
    ssgaSky   *theSky;
    float     radius;
    sgdVec3   planetsDummy;
    sgdVec3   starsDummy;
    sgVec4    skyDomeCol;
    sgVec4    skyFogCol;
    sgVec4    skyCloudCol;

    void      repaint();

  public:
    SkyDome(SimpleXMLTransfer *xml, float r,
            sgVec4 domecol, sgVec4 fogcol, sgVec4 cloudcol);
    ~SkyDome();

    /// Draw the sky dome
    void preDraw() {theSky->preDraw();}
  
    /// Draw the clouds
    void postDraw(float altitude) {theSky->postDraw(altitude);}
    
    /// Update the camera position and cloud movement
    void update(sgVec3 *campos, double dt);
};
#endif

/* ******************************************************** */

/** \brief CRRCsim's original sky dome
 *
 *  This class creates a textured sky dome using SSG.
 */
class CRRCSkyDome : public SkyRenderer
{
  private:
    ssgRoot       *skyroot;
    ssgTransform  *skyboxtrans;
    float         radius;
  
    void repaint(const char *texture);

  public:
    CRRCSkyDome(SimpleXMLTransfer *xml, const char *cloud_texture, float r = 8000.0);
    ~CRRCSkyDome();
  
    /// Update the camera position and cloud movement
    void update(sgVec3 *campos, double dt);
  
    /// Draw the sky dome
    void preDraw(); 
};


/* ******************************************************** */

#define PANO_NUM_RINGS     10
#define PANO_NUM_SLICES    20
#define PANO_RENDER_MODE   1
#define PANO_CAM_HEIGHT    4
/** \brief A panorama renderer for CRRCsim
 *
 *  This class renders a 360*135 deg panorama dome.
 */
class CRRCPanoDome : public SkyRenderer
{
  private:
    void init_textures();
    void init_vertices();
    void render();

    std::string     texfilename;
    ssgTexture      *panotex;
    ssgSimpleState  *state;
    float           radius;
    sgVec3 vertices[PANO_NUM_RINGS][PANO_NUM_SLICES];
    sgVec2 texco[PANO_NUM_RINGS][PANO_NUM_SLICES];
  
  public:
    CRRCPanoDome(SimpleXMLTransfer *xml, const char *texture, float r = 10.0);
    ~CRRCPanoDome();
  
    /// Draw the panorama
    void preDraw();
};

} // namespace Video

#endif  // _CRRC_SKY_H
