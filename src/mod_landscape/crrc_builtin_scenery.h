/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 *   Copyright (C) 2000, 2001 Jan Kansky (original author)
 *   Copyright (C) 2004-2010 Jan Reucker
 *   Copyright (C) 2005, 2008 Jens Wilhelm Wulf
 *   Copyright (C) 2009 Joel Lienard
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

#ifndef CRRC_BUILTIN_SCENERY_H
#define CRRC_BUILTIN_SCENERY_H

#include <crrc_config.h>
#include "crrc_scenery.h"
#include "../include_gl.h"

#define HEIGHTMAP_SIZE_X  (64)
#define HEIGHTMAP_SIZE_Z  (64)


/** \brief Class for creating and drawing the terrain.
 *
 *
 */
class BuiltinScenery : public Scenery
{
  public:
    /**
     *  Constructor that initializes the object from
     *  a SimpleXMLTransfer object.
     */
    BuiltinScenery(SimpleXMLTransfer *xml, int sky_variant = 0, bool boIsNullRenderer = false);

    /**
     *  Constructor for old XML_HMAP format.
     *  \todo Make it work properly or delete it.
     */
    BuiltinScenery(const char *mapfile);
  
    /**
     *  Destructor
     */
    ~BuiltinScenery();

    /**
     *  Draw the scenery
     */
    virtual void draw(double current_time) = 0;

    // Draw normal vectors for debugging purposes
    void draw_normals(float length);
  
    /**
     *  Set texturing mode.
     *
     *  \todo Does this still make sense? Not used right now.
     *  Switching on the textures won't work if they were turned
     *  off while the object was created. Make it work or
     *  delete it.
     */
    void setTextures(bool yesno);
  
    /** 
     *  Get the terrain height at (x|z). Must be implemented
     *  by the child classes.
     *  \param x x coordinate
     *  \param z z coordinate
     *  \return terrain height at this point in ft
     */
    virtual float getHeight(float x, float z) = 0;
    
    /**
     *  Get height and plane equation of the terrain at (x|z).
     *  Must be implemented by the child classes.
     *  \param x x coordinate
     *  \param z z coordinate
     *  \param tplane this is where the plane equation will be stored
     *  \return terrain height at this point in ft
     */
    virtual float getHeightAndPlane(float x, float z, float tplane[4]) = 0;

    /**
     *  Get an ID code for this location or scenery type
     *  Must be implemented by the child classes.
     */
    virtual int getID() = 0;

    /**
     *  Get wind components at position X_cg, Y_cg, Z_cg
     */
    virtual int getWindComponents(double X_cg, double Y_cg, double Z_cg,
                                  float *x_wind_velocity, 
                                  float *y_wind_velocity, 
                                  float *z_wind_velocity) = 0;

  protected:
    int use_textures;
    GLUquadricObj *quadric; ///\todo remove dependencies to GLUT

    void setup_drawing_state();
    void restore_drawing_state();
    
  private:
    void calculate_normals();
    void compile_display_list();
  
    unsigned int list;
  
    float size_x;
    float size_z;
    float offset_x;
    float offset_z;
    float alt_min;
    float alt_max;
    CRRCMath::Vector3 model_start;
    CRRCMath::Vector3 player;

    float height[HEIGHTMAP_SIZE_X][HEIGHTMAP_SIZE_Z];
    CRRCMath::Vector3 normal[HEIGHTMAP_SIZE_X][HEIGHTMAP_SIZE_Z];
};


/** \brief Class for rendering the built-in Davis field
 *
 *  This class renders the original built-in Davis field
 *  scenery.
 */
class BuiltinSceneryDavis : public BuiltinScenery
{
  public:
    BuiltinSceneryDavis(SimpleXMLTransfer *xml, int sky_variant = 0);
    ~BuiltinSceneryDavis();

    /** 
     *  Get the terrain height at (x|z).
     *  \param x x coordinate
     *  \param z z coordinate
     *  \return terrain height at this point in ft
     */
    float getHeight(float x, float z);
    
    /**
     *  Get height and plane equation of the terrain at (x|z).
     *  \param x x coordinate
     *  \param z z coordinate
     *  \param tplane this is where the plane equation will be stored
     *  \return terrain height at this point in ft
     */
    float getHeightAndPlane(float x, float z, float tplane[4]);

    /**
     *  Get an ID code for this location or scenery type
     */
    int getID() {return location;};

    /**
     *  Get wind components at position X_cg, Y_cg, Z_cg
     */
    int getWindComponents(double X_cg,double Y_cg,double Z_cg,
                          float *x_wind_velocity, 
                          float *y_wind_velocity,
                          float *z_wind_velocity);

    /**
     *  Draw the scenery
     *  \param current_time current time in ms (for animation effects)
     */
    void draw(double current_time);

    
  private:
    void read_textures(SimpleXMLTransfer *xml);
    void clear_textures();
    int location;

    unsigned char *ground_texture;
    int ground_texture_width;
    int ground_texture_height;
    unsigned char *grass_texture;
    int grass_texture_width;
    int grass_texture_height;
    unsigned char *grass_side_texture;
    int grass_side_texture_width;
    int grass_side_texture_height;
    unsigned char *grass_top_texture;
    int grass_top_texture_width;
    int grass_top_texture_height;
    unsigned char *eastern_view_texture;
    int eastern_view_texture_width;
    int eastern_view_texture_height;
    unsigned char *netrees_texture;
    int netrees_texture_width;
    int netrees_texture_height;
    unsigned char *dirt_texture;
    int dirt_texture_width;
    int dirt_texture_height;
    unsigned char *outhouse_texture;
    int outhouse_texture_width;
    int outhouse_texture_height;
    unsigned char *freq_texture;
    int freq_texture_width;
    int freq_texture_height;
    unsigned char *pine_texture;
    int pine_texture_width;
    int pine_texture_height;
    unsigned char *decid_texture;
    int decid_texture_width;
    int decid_texture_height;
    GLuint groundTexture;          // GL ground texture handle
    GLuint grassTexture;          // GL grass texture handle
    GLuint grassSideTexture;          // GL grass Side texture handle
    GLuint grassTopTexture;          // GL grass Top texture handle
    GLuint pineTexture;          // GL grass texture handle
    GLuint decidTexture;          // GL grass texture handle
    GLuint easternViewTexture;          // Looking to the east
    GLuint netreesTexture;          // Looking to the east
    GLuint dirtTexture;          // Looking to the east
    GLuint outhouseTexture;          // Looking to the east
    GLuint freqTexture;          // Looking to the east
};


/** \brief Class for rendering the built-in Cape Cod slope
 *
 *  This class renders the original built-in Cape Cod slope
 *  scenery.
 */
class BuiltinSceneryCapeCod : public BuiltinScenery
{
  public:
    BuiltinSceneryCapeCod(SimpleXMLTransfer *xml, int sky_variant = 0);
    ~BuiltinSceneryCapeCod();

    /** 
     *  Get the terrain height at (x|z).
     *  \param x x coordinate
     *  \param z z coordinate
     *  \return terrain height at this point in ft
     */
    float getHeight(float x, float z);
    
    /**
     *  Get height and plane equation of the terrain at (x|z).
     *  \param x x coordinate
     *  \param z z coordinate
     *  \param tplane this is where the plane equation will be stored
     *  \return terrain height at this point in ft
     */
    float getHeightAndPlane(float x, float z, float tplane[4]);

    /**
     *  Get an ID code for this location or scenery type
     */
    int getID() {return location;};

    /**
     *  Get wind components at position X_cg, Y_cg, Z_cg
     */
    int getWindComponents(double X_cg,double Y_cg,double Z_cg,
                          float *x_wind_velocity, 
                          float *y_wind_velocity,
                          float *z_wind_velocity);
  
    /**
     *  Draw the scenery
     */
    void draw(double current_time);
    
  private:
    int location;
    void read_textures(SimpleXMLTransfer *xml);
    void clear_textures();

    unsigned char *water_texture;
    int water_texture_width;
    int water_texture_height;
    unsigned char *beachsand_texture;
    int beachsand_texture_width;
    int beachsand_texture_height;
    unsigned char *scrub_texture;
    int scrub_texture_width;
    int scrub_texture_height;
    unsigned char *scrubedge_texture;
    int scrubedge_texture_width;
    int scrubedge_texture_height;
    unsigned char *south_texture;
    int south_texture_width;
    int south_texture_height;
    unsigned char *hilledge_texture;
    int hilledge_texture_width;
    int hilledge_texture_height;
    unsigned char *waves_texture;
    int waves_texture_width;
    int waves_texture_height;
    unsigned char *sand_texture;
    int sand_texture_width;
    int sand_texture_height;
    GLuint waterTexture;
    GLuint beachsandTexture;
    GLuint scrubTexture;
    GLuint scrubedgeTexture;
    GLuint southTexture;
    GLuint hilledgeTexture;
    GLuint wavesTexture;
    GLuint sandTexture;
};


#endif  // CRRC_BUILTIN_SCENERY_H
