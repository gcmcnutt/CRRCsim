/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005-2010 Jan Reucker (original author)
 * Copyright (C) 2005, 2006, 2007, 2008 Jens Wilhelm Wulf
 * Copyright (C) 2008 Olivier Bordes
 * Copyright (C) 2009 Joel Lienard
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
  

/** \file crrc_graphics.cpp
 *
 *  Functions dealing with graphics.
 */

#include <errno.h>
#include "../i18n.h"
#include "../global.h"
#include "../aircraft.h"
#include "../SimStateHandler.h"
#include "../mod_mode/T_GameHandler.h"
#include "crrc_graphics.h"
#include "../crrc_main.h"
#include "../GUI/crrc_gui_main.h"
#include "../mod_inputdev/inputdev_audio/inputdev_audio.h"
#include "../mod_windfield/windfield.h"
#include "../crrc_loadair.h"
#include "../mod_misc/lib_conversions.h"
#include "../crrc_system.h"
#include "../defines.h"
#include "../mod_landscape/crrc_scenery.h"
#include "glconsole.h"
#include "gloverlay.h"
#include "../zoom.h"
#include "fonts.h"
#include "../mod_misc/filesystools.h"

// model close-view tuning parameters
#define MODEL_VIEW_SIZE        0.15 // fraction of screen used up by close-view window
#define MODEL_VIEW_FOV_MARGIN  0.3  // extra FOV width (relative)

// smart camera tuning parameters
#define SMARTCAM_MAX_FRACTION  0.6 // maximum fraction of screen used up by smart camera
#define SMARTCAM_TIME          60. // smart camera adaptation time (s)
#define SMARTCAM_RANGE_FAC     1.3 // smart camera range = 4/3 sigma

// Debug and error handling settings
#define DONT_REPEAT_GL_ERRORS  1


namespace Video
{
bool ssgLoadJPG(const char *fname, ssgTextureInfo* info);

fntTexFont   *textureFont; //textured font 
std::string codeSet; //for font filename construct

// Flags for SDL video mode
Uint32 SDL_video_flags;

static ssgContext *context = NULL ;

int window_xsize;  // Size of window in x direction
int window_ysize;  // Size of window in y direction
int screen_xsize;  // Size of screen in x direction
int screen_ysize;  // Size of screen in y direction

T_VideoBitDepthInfo vidbits;

/**
 * Smart camera setting
 * and smart camera center/range
 */
float flSmartCam = SMARTCAM_MAX_FRACTION;
float flSmartCam_time;
float flSmartCam_center_w;
float flSmartCam_center_h;
float flSmartCam_var_w;
float flSmartCam_var_h;

/**
 * Sloppy camera setting
 */
float flSloppyCam = 0;

/**
 * Position where the player is looking at (may be different
 * from position of plane; needed for sloppy & smart camera).
 */
CRRCMath::Vector3 looking_pos;

/**
 * The console overlay
 */
static GlConsole* console = NULL;


/** \brief get a pointer to the global rendering context
 *
 * \return pointer to global rendering context
 */
ssgContext* getGlobalRenderingContext()
{
  return context;
}


/** \brief Dump some info about the current state of the stacks
 *
 *  \param pFile the stream to print the info to
 */
void dumpGLStackInfo(FILE* pFile)
{
  GLint mv_cur, mv_max;
  GLint pr_cur, pr_max;
  
  glGetIntegerv(GL_MODELVIEW_STACK_DEPTH, &mv_cur);
  glGetIntegerv(GL_PROJECTION_STACK_DEPTH, &pr_cur);
  glGetIntegerv(GL_MAX_MODELVIEW_STACK_DEPTH, &mv_max);
  glGetIntegerv(GL_MAX_PROJECTION_STACK_DEPTH, &pr_max);
  
  fprintf(pFile, "Modelview stack: %d / %d    Projection stack: %d / %d\n",
          mv_cur, mv_max, pr_cur, pr_max);
}


/** \brief Evaluate any pending OpenGL error 
 *
 *  Prints an error message if the OpenGL state machine
 *  error flag is set to any value other than GL_NO_ERROR.
 *  
 *  \return true if an OpenGL error occured
 */
bool evaluateOpenGLErrors()
{
  static GLenum lastGLerror = GL_NO_ERROR;
  bool has_error = false;
  GLenum err;

  if ((err = glGetError()) != GL_NO_ERROR)
  {
    #if DONT_REPEAT_GL_ERRORS > 0
    if (err != lastGLerror)
    #endif
    {
      const GLubyte *s = gluErrorString(err);
      if (s != NULL)
      {
        fprintf(stderr, "OpenGL error: %s\n", s);
      }
      Video::dumpGLStackInfo(stderr);
      lastGLerror = err;
    }
    has_error = true;
  }
  return has_error;
}


/*****************************************************************************/
unsigned short getshort(FILE *inf)
{
  unsigned char buf[2];
  
  int nres = fread(buf,2,1,inf);
  if (nres != 1) return 0;
  return (buf[0]<<8)+(buf[1]<<0);
}

/*****************************************************************************/
unsigned short getrgbchar(FILE *inf)
{
  unsigned char buf[1];

  int nres = fread(buf,1,1,inf);
  if (nres != 1) return 0;
  return (buf[0]);
}


/** \brief Read pixel data from an SGI .rgb image.
 *
 *  Reads an .rgb file, allocates memory for the pixels
 *  and sets *w and *h to image width and height.
 *
 *  \param name file name
 *  \param w will be set to image width
 *  \param h will be set to image height
 *  \return pointer to the pixel data or NULL on error
 */
unsigned char * read_rgbimage(const char *name, int *w, int *h)
{
  unsigned char *image, *temp;
  FILE *image_in;
  unsigned char input_char;
  unsigned short int input_short;
  unsigned char header[512];
  unsigned long int loop;

  if ( (image_in = fopen(name, "rb")) == NULL)
  {
    printf("%s\n", name);
    std::string s = "read_rgbimage: Unable to open ";
    s += name;
    perror(s.c_str());
    return NULL;
  }
  input_short=getshort(image_in);
  if (input_short == 0x01da)
  {
    input_char=getrgbchar(image_in);
    if (input_char == 0)
    {
      input_char=getrgbchar(image_in);
      input_short=getshort(image_in);
      if (input_short == 3)
      {
        input_short=getshort(image_in);
        *w=input_short;
        input_short=getshort(image_in);
        *h=input_short;
        image=(unsigned char*)malloc(*w * *h *4 *sizeof(unsigned char));
        temp=(unsigned char*)malloc(*w * *h *sizeof(unsigned char));
        if ((image == NULL) || (temp == NULL))
        {
          fprintf(stderr, "Error allocating memory for %s\n", name);
          // just in case one of the two was malloc'ed correctly:
          free(image);
          free(temp);
          return NULL;
        }
        input_short=getshort(image_in);
        if (input_short == 4)
        {
          int nres = fread(header,sizeof(unsigned char),500,image_in);
          if (nres != 500) return NULL;
          nres = fread(temp, sizeof image[0], *w * *h, image_in);
          if (nres != (*w * *h)) return NULL;
          for (loop=0;loop<(unsigned long int)(*w * *h);loop++)
          {
            image[loop*4+0]=temp[loop];
          }
          nres = fread(temp, sizeof image[0], *w * *h, image_in);
          if (nres != (*w * *h)) return NULL;
          for (loop=0;loop<(unsigned long int)(*w * *h);loop++)
          {
            image[loop*4+1]=temp[loop];
          }
          nres = fread(temp, sizeof image[0], *w * *h, image_in);
          if (nres != (*w * *h)) return NULL;
          for (loop=0;loop<(unsigned long int)(*w * *h);loop++)
          {
            image[loop*4+2]=temp[loop];
          }
          nres = fread(temp, sizeof image[0], *w * *h, image_in);
          if (nres != (*w * *h)) return NULL;
          for (loop=0;loop<(unsigned long int)(*w * *h);loop++)
          {
            image[loop*4+3]=temp[loop];
          }
          free(temp);
          return image;
        }
        else
        {
          std::string s = "Error loading texture ";
          s += name;
          s += ":\nThis file isn't a 4 channel RGBA file.";
          fprintf(stderr, "%s\n", s.c_str());
          return NULL;
        }
      }
      else
      {
        std::string s = "Error loading texture ";
        s += name;
        s += ":\nNot a useable RGB file.";
        fprintf(stderr, "%s\n", s.c_str());
        return NULL;
      }
    }
    else
    {
      std::string s = "Error loading texture ";
      s += name;
      s += ":\nRLE encoded SGI files are not supported.";
      fprintf(stderr, "%s\n", s.c_str());
      return NULL;
    }
  }
  else
  {
    std::string s = "Error loading texture ";
    s += name;
    s += ":\nFile doesn't appear to be an SGI rgb file!";
    fprintf(stderr, "%s\n", s.c_str());
    return NULL;
  }
  return NULL;
}


/**
 * <code>w</code> and <code>h</code> need to be set before calling
 * this function.
 *
 * \todo .bw images are always square, so we can calculate
 * <code>w</code> and <code>h</code> from the number of read bytes.
 */
unsigned char * read_bwimage(const char *name, int *w, int *h)
  // From Skyfly.c, Thanks dude
{
  unsigned char   *image;
  FILE            *image_in;
  int             img_bytes;
  int             bytes_read;

  if ( (image_in = fopen(name, "rb")) == NULL)
  {
    return NULL;
  }

  img_bytes = *w * *h;
  image = (unsigned char *)malloc(sizeof(unsigned char) * img_bytes);

  bytes_read = fread(image, sizeof image[0], img_bytes, image_in);
  fclose(image_in);
  if (bytes_read != img_bytes)
  {
    free(image);
    return NULL;
  }
  return image;
}


/*****************************************************************************/
/** \brief Create a texture from raw pixel data.
 *
 *  Generates an OpenGL texture from raw image data which must
 *  be loaded before using <code>read_XXXimage</code>. The
 *  texture is completely independent of the raw data, so you
 *  can free() the pixel_data after generating the texture.
 *
 *  \param pixel_data   pointer to the raw pixel data
 *  \param pixel_format format of the pixel data (e.g. GL_ALPHA for 8 bpp images,
 *                      GL_RGB for 24 bpp images and GL_RGBA for 32 bpp images)
 *  \param format format of the texture to be generated (e.g. GL_LUMINANCE,
 *                GL_ALPHA, GL_RGBA, ...)
 *  \param width  width of the original image
 *  \param height height of the original image
 *  \param use_mipmaps specify whether or not to generate mipmaps
 *  \return OpenGL texture handle
 */
GLuint make_texture(unsigned char *pixel_data, GLint pixel_format, GLint format,
                    GLsizei width, GLsizei height, bool use_mipmaps)
{
  GLuint tex;

  glGenTextures(1, &tex);
  glBindTexture(GL_TEXTURE_2D, tex);
  glPixelStorei(GL_UNPACK_ALIGNMENT,1);
  glTexEnvf(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
  if (!use_mipmaps)
  {
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, pixel_format, width, height,
                 0, format, GL_UNSIGNED_BYTE, pixel_data);
  }
  else
  {
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_LINEAR);
    gluBuild2DMipmaps(GL_TEXTURE_2D, pixel_format, width,
                      height, format, GL_UNSIGNED_BYTE,
                      pixel_data);
  }

  return tex;
}


/** \brief Perform the basic scenegraph initialization
 *
 *  Initializes the PLIB SSG library, creates the scenegraph
 *  root, a rendering context and the sun.
 */
void initialize_scenegraph()
{
  // Initialize SSG
  ssgInit();
  
  // add to SSG function for read JPEG Textures 
  ::ssgAddTextureFormat(".jpg", ssgLoadJPG);
  
  // font
  {
    #define FONT_FILE_PREFIX  "textures/Sans_"
    #define FONT_FILE_SUFFIX  ".txf"
    std::string name = FONT_FILE_PREFIX+codeSet+FONT_FILE_SUFFIX;
    textureFont = new fntTexFont ();
    std::string fname = FileSysTools::getDataPath(name.c_str());
    if ((fname == "") || 
        (textureFont->load( fname.c_str(),  GL_LINEAR, GL_LINEAR ) == FNT_FALSE))
    {
      std::cout << "Unable to find font " << name << "("<<fname<< "), falling back to bitmap font!" << std::endl;
      puSetDefaultFonts ( FONT_HELVETICA_14, FONT_HELVETICA_14 );
      textureFont = NULL;
    }
    else
    {
      std::cout << "Textured font :" << fname<< std::endl;
      textureFont->setGap(0);
      puFont *myfont;
      myfont  = new puFont ( textureFont, 15 ) ;
      puSetDefaultFonts ( *myfont, *myfont );
    }
  }
  
  // Some basic OpenGL setup
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glEnable(GL_DEPTH_TEST);

  // TWO_SIDE light model is required to correctly display
  // thin two-sided surfaces (e.g. in airplane models)
  glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, 1.0);

  // Set up the viewing parameters
  context = new ssgContext();
  context->setFOV     ( 35.0f, 0 ) ;
  context->setNearFar ( 1.0f, 10000.0f ) ;
  context->makeCurrent();

  ssgModelPath("");
  ssgTexturePath("textures");  
}


void adjust_zoom(float field_of_view)
{
  // Skip OpenGL operations in headless mode
  if (cfgfile->getInt("video.enabled", 1) == 0)
    return;
    
  float aspect = (GLfloat)window_xsize/(GLfloat)window_ysize;
  float near_dist = 1.0;
  float far_dist = 10000.0;
  
  glViewport(0,0,(GLsizei)window_xsize,(GLsizei)window_ysize);

  // setup perspective for non-SSG rendering
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(field_of_view/aspect,aspect,near_dist,far_dist);
  glMatrixMode(GL_MODELVIEW);

  // setup perspective for SSG rendering
  context->setFOV(field_of_view,field_of_view/aspect);
  context->setNearFar(near_dist,far_dist);
}


void adjust_model_zoom(float size_ratio, float field_of_view, float distance)
{
  int model_window_xsize = window_xsize*size_ratio;
  int model_window_ysize = window_ysize*size_ratio;
  float aspect = (GLfloat)model_window_xsize/(GLfloat)model_window_ysize;
  float near_dist = distance - 50.0;
  near_dist = near_dist > 1.0 ? near_dist : 1.0;
  float far_dist = near_dist + 100.0;
  
  glViewport(0,(GLsizei)(window_ysize-model_window_ysize),
      (GLsizei)model_window_xsize,(GLsizei)model_window_ysize);
  glScissor(0,(GLsizei)(window_ysize-model_window_ysize),
      (GLsizei)model_window_xsize,(GLsizei)model_window_ysize);
  glEnable(GL_SCISSOR_TEST);

  // setup perspective for non-SSG rendering
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(field_of_view/aspect,aspect,near_dist,far_dist);
  glMatrixMode(GL_MODELVIEW);

  // setup perspective for SSG rendering
  context->setFOV(field_of_view,field_of_view/aspect);
  context->setNearFar(near_dist,far_dist);
}


/***********************************************/
#define NPT_OSCILLO 500
void oscillo()
{
#if PORTAUDIO > 0
  // TEST_MODE
  int i;

#if 0
  glDisable(GL_LIGHTING);
  glMatrixMode (GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity ();
  gluOrtho2D (0, window_xsize-1, 0, window_ysize);
#endif
  glTranslatef(10,10,0);
  glColor3f (0.1, 0.2, 0.2); //oscillo border
  glRectf(-5,-5,NPT_OSCILLO+5,200+5);
  glTranslatef(0,0,0.1);
  glColor3f (0, 0, 0);  //oscillo screen
  glRectf(0,0,NPT_OSCILLO,200);
  glTranslatef(0,0,0.1);

  glColor3f (1, 0., 0.); //oscillo border
  glBegin(GL_LINE_LOOP);
  glVertex2i(0,0);
  glVertex2i(0,200);
  glVertex2i(NPT_OSCILLO,200);
  glVertex2i(NPT_OSCILLO,0);
  glEnd();
  glColor3f (0, 1, 0.);
  glBegin(GL_LINE_STRIP);
  for(i = 0; i < NPT_OSCILLO; i++)
  {
    //draw curve
    int v;
    v = (int)( 100+ 100. * get_audio_signal(i));
    glVertex2i(i,v);
  }
  glEnd();
#if 0
  glPopMatrix();
  glEnable(GL_LIGHTING);
#endif

#endif
}


/**
 * Converts a vector from FDM to graphics representation
 */
CRRCMath::Vector3 FDM2Graphics(CRRCMath::Vector3 const& v)
{
  // This coordinate transformation works in both headless and video modes
  return CRRCMath::Vector3(v.r[1], -v.r[2], -v.r[0]);
}


/**
 * Converts a vector from graphics to FDM representation
 */
CRRCMath::Vector3 Graphics2FDM(CRRCMath::Vector3 const& v)
{
  return CRRCMath::Vector3(-v.r[2], v.r[0], -v.r[1]);
}


/*****************************************************************************/
/** \brief The per-frame OpenGL display routine
 *
 *  This function does the complete OpenGL drawing. First
 *  the 3D scene is drawn, then some 2D overlays
 *  (wind and battery indicator, GUI).
 */
void display()
{
  // Skip all rendering in headless mode
  if (cfgfile->getInt("video.enabled", 1) == 0)
    return;
    
  CRRCMath::Vector3 plane_pos = FDM2Graphics(Global::aircraft->getPos());

  // Prepare the current frame buffer and reset
  // the modelview matrix (for non-SSG drawing)
  GLbitfield clearmask = GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT;
  if (vidbits.stencil)
  {
    clearmask |= GL_STENCIL_BUFFER_BIT;
  }
  glClear(clearmask);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  //~ glBlendFunc(GL_ONE, GL_ZERO);
  //~ glTexEnvf(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);

  // 3D scene
  if (Global::scenery->getID() != 0)
  {
    glPushMatrix();

    // Set up the viewing transformation (for SSG drawing)
    sgVec3 vPlayerPos, vLookingPos, vUp;
    sgSetVec3(vPlayerPos, player_pos.r[0], player_pos.r[1], player_pos.r[2]);                
    sgSetVec3(vLookingPos, looking_pos.r[0], looking_pos.r[1], looking_pos.r[2]);
    sgSetVec3(vUp, 0.0, 1.0, 0.0);  
    context->setCameraLookAt(vPlayerPos, vLookingPos, vUp);

    // Set up ambient light and sun light for SSG rendering,
    // according to scenery's sky definition
    sgVec4 ambient_light;
    Global::scenery->getAmbientColor(ambient_light);
    sgVec4 sun_light;
    Global::scenery->getSunColor(sun_light);
    sgVec3 sun_pos;
    Global::scenery->getSunPosition(sun_pos);
    ssgGetLight(0)->setColour(GL_AMBIENT, ambient_light);
    ssgGetLight(0)->setColour(GL_DIFFUSE, sun_light);
    ssgGetLight(0)->setColour(GL_SPECULAR, sun_light);
    ssgGetLight(0)->setPosition(sun_pos);

    // Set up the fog, according to scenery's sky definition
    float fog = Global::scenery->getFogDensity();
    sgVec4 fog_color;
    Global::scenery->getFogColor(fog_color);
    glHint(GL_FOG_HINT, GL_NICEST);   //set the fog to look nicest
    glFogi(GL_FOG_MODE, GL_EXP2);     //set the fog mode
    glFogfv(GL_FOG_COLOR, fog_color);
    glFogf(GL_FOG_DENSITY, fog);

    // 3D scene: sky
    glDisable(GL_FOG);
    draw_sky(&vPlayerPos, Global::Simulation->getSimulationTimeSinceReset());

    // only enable fog after sky has been rendered
    if (fog > 0)
      glEnable(GL_FOG);
    
    // 3D scene: scenery
    // This shall be drawn first so that transparent terrain
    // (used in model-based photo realistic sceneries) hides
    // the objects drawn successively (e.g. airplane)
    Global::scenery->draw(Global::Simulation->getSimulationTimeSinceReset());
  
    // 3D scene: airplanes
    // Draw all airplane models, including robots, and their shadows
    draw_all_airplanes(Global::Simulation->getSimulationTimeSinceReset());
    
    // 3D scene: casted shadows (by all objects in the scene)    
#if (SHADOW_TYPE==SHADOW_VOLUME)
    // mark casted shadow in thr stencil buffer
    Global::scenery->draw_shadows(Global::Simulation->getSimulationTimeSinceReset());
    draw_all_airplanes(Global::Simulation->getSimulationTimeSinceReset(), false);
    
    // draw casted shadows marked in the stencil buffer
    shadowVolumeDrawShadows();
#endif

    // ssgCullAndDraw() ends up with an identity modelview matrix,
    // so we have to set up our own viewing transformation to
    // properly rendere following optional items.
    // NB: light & fog are left unchanged, so they are still ok.
    gluLookAt(player_pos.r[0], player_pos.r[1], player_pos.r[2],
              looking_pos.r[0], looking_pos.r[1], looking_pos.r[2],
              0.0, 1.0, 0.0);
    context->forceBasicState();
    
    // 3D scene: game-mode-specific stuff (pylons etc.)
    Global::gameHandler->draw();

    // disable the fog when finished rendering the scene
    glDisable(GL_FOG);

    // 3D scene: optionally draw thermals
    if (Global::training_mode == TRUE)
    {
      draw_thermals(Global::aircraft->getPos());
    }
    
    // 3D scene: optionally draw wind vectors
    if (Global::windVectors > 0)
    {
      Global::scenery->drawWindField(Global::aircraft->getPos(), Global::windVectors);
    }
  
    // 3D scene: optionally draw model-view window
    if (!Global::testmode && Global::modelView > 0)
    {
      // get half the aircraft size and aircraft position
      float aircraftSize = Global::aircraft->getModel()->getAircraftSize();
      CRRCMath::Vector3 aircraft_pos = FDM2Graphics(Global::aircraft->getPos());
      
      // compute model-view field of view    
      // remember: tan(fieldOfView/2) * distance = aircraftSize
      float distance_to_model = (aircraft_pos - player_pos).length();
      float modelFieldOfView = 180.0/M_PI*2.*atan(aircraftSize/distance_to_model);
      modelFieldOfView *= (1.0 + MODEL_VIEW_FOV_MARGIN);
      
      float fieldOfView = zoom_get();
      float modelViewSize = MODEL_VIEW_SIZE;
      float fov_ratio = fieldOfView*modelViewSize/modelFieldOfView;
      float fov_ratio_limit = 1.0;
      
      // draw model-view window if required or convenient (in auto mode)
      if (Global::modelView == 1 || fov_ratio > fov_ratio_limit)
      {
        glPushAttrib(GL_VIEWPORT_BIT);
        glPushAttrib(GL_ENABLE_BIT | GL_TRANSFORM_BIT | GL_LIGHTING_BIT);
        
        // set viewport  
        adjust_model_zoom(modelViewSize, modelFieldOfView, distance_to_model);

        // fill the viewport with a translucent white bkg
        glMatrixMode(GL_PROJECTION);
        glPushMatrix(); 
        glLoadIdentity();
        gluOrtho2D(0.0,1.0,0.0,1.0);
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();
        glDisable(GL_LIGHTING);
        glDisable(GL_TEXTURE_2D);
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glBegin(GL_QUADS);
        glColor4f(1.0,1.0,1.0,0.5);
        glVertex2f(0.0,0.0);
        glVertex2f(1.0,0.0);
        glVertex2f(1.0,1.0);
        glVertex2f(0.0,1.0);
        glEnd();      
        
        // restore original settings but for viewport
        glPopAttrib();
        
        // Set up the viewing transformation (for SSG drawing)
        // camera looking straight at the model
        sgVec3 vPlanePos;
        sgSetVec3(vPlanePos, plane_pos.r[0], plane_pos.r[1], plane_pos.r[2]);                
        context->setCameraLookAt(vPlayerPos, vPlanePos, vUp);
              
        // clear zbuffer in the close-view window
        glClearDepth(1.0);
        glClear(GL_DEPTH_BUFFER_BIT);
         
        // draw the airplane model, without any casted shadow
        Global::aircraft->getModel()->draw();

        // restore original viewport and camera
        glPopAttrib();
        glMatrixMode(GL_PROJECTION);
        glPopMatrix(); 
        glMatrixMode(GL_MODELVIEW);
        glPopMatrix();

        context->forceBasicState();
      }
    }
  
    glPopMatrix();
  }

  // Overlay: scope for audio interface
  if (Global::testmode
       &&
       (Global::TXInterface->inputMethod() == T_TX_Interface::eIM_audio))
  {
    GlOverlay::setupRenderingState(window_xsize, window_ysize);
    oscillo();
    GlOverlay::restoreRenderingState();
  }

  // Overlay: wind direction indicator
  {
    double dx  = (plane_pos.r[2] - player_pos.r[2]);
    double dy  = (player_pos.r[0] - plane_pos.r[0]);
    double dir = atan2(dy, dx);

    GlOverlay::setupRenderingState(window_xsize, window_ysize);
    draw_wind(dir);
    GlOverlay::restoreRenderingState();
  }

  // Overlay: battery capacity/fuel left
  {
    int r   = window_ysize >> 5;
    int w   = r >> 1;
    int h   = window_ysize >> 3;
    int ht  = (int)(Global::aircraft->getFDM()->getBatCapLeft() * h);
                    
#if 0
    glDisable(GL_LIGHTING);
    glMatrixMode (GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity ();
    gluOrtho2D (0, window_xsize-1, 0, window_ysize);
#endif
    GlOverlay::setupRenderingState(window_xsize, window_ysize);
    
    // Background
    glColor3f (0, 0, 0);
    glRectf(window_xsize-w, r+ht,
            window_xsize-1, r+h);
    glTranslatef(0,0,0.1);
  
    // Indicator
    glColor3f (0, 1, 0.);
    glRectf(window_xsize-w, r,
            window_xsize-1, r+ht);
    
#if 0
    glPopMatrix();
    glEnable(GL_LIGHTING);    
    glMatrixMode(GL_MODELVIEW);
#endif
    GlOverlay::restoreRenderingState();
  }

  // Overlay: game handler
  Global::gameHandler->display_infos(window_xsize, window_ysize);

  // Overlay: console
  console->render(window_xsize, window_ysize);
  
  // Overlay: gui
  Global::gui->draw();
  
  // check for any OpenGL errors
  evaluateOpenGLErrors();

  // Force pipeline flushing and flip front and back buffer
  glFlush();
  SDL_GL_SwapBuffers();
}


/*****************************************************************************/
#if 0
// This was a test for Win32, didn't work...
void rebuild_video_context(int w, int h)
{
  printf("rebuilding context in %d x %d\n", w, h);
  if (SDL_WasInit(SDL_INIT_VIDEO))
    SDL_QuitSubSystem(SDL_INIT_VIDEO);

  if (SDL_InitSubSystem(SDL_INIT_VIDEO) != 0)
  {
    crrc_exit(CRRC_EXIT_FAILURE, SDL_GetError());
  }

  printf("Setting up new surface...\n");
  setupScreen(w, h, cfgfile->getInt("video.fullscreen.fUse", 0));
  delete scenery;
  //  delete SkySphere;
  printf("Setting up new Sky...\n");
  SkySphere = new CSkySphere(8000.0,
                             cfgfile->getInt("video.textures.fUse_textures", 1));
  printf("Setting up new scenery...\n");
  scenery = new BuiltinScenery(cfg->location());
  if (read_airplane_file(cfgfile->getString("airplane.file", "models/allegro.xml").c_str()))
  {
    // Failed to load airplane file.
    // Using some fallback.
    cfgfile->setAttributeOverwrite("airplane.file", FileSysTools::getDataPath("models/allegro.xml"));

    if (read_airplane_file(cfgfile->getString("airplane.file").c_str()))
    {
      // The fallback failed, too. Abort.
      fprintf(stderr, "Unable to load airplane file %s\n", cfgfile->getString("airplane.file").c_str()));
      crrc_exit(CRRC_EXIT_FAILURE, "Unable to load airplane file");
    }
  }
  setWindowTitleString();
  initialize_window(cfg);
}
#endif


/** \brief Add an OpenGL string to a std::string
 *
 *  Reads a predefined OpenGL info string and adds
 *  it to the string or adds "<unknown>" if the
 *  result was NULL.
 *
 *  \param s String
 *  \param which Symbolic name of the GL string
 */
void AddGLString(std::string& s, GLenum which)
{
  const GLubyte *str;
  
  str = glGetString(which);
  if (str != NULL)
  {
    s += (const char *)str;
  }
  else
  {
    s += "<unknown>";
  }
}


/** \brief Get info string for current video mode
 *
 *  This function returns a std::string containing
 *  some information about the current video context,
 *  e.g. driver information, bpp for different buffers...
 *
 *  \param indent string to be applied at the beginning of each line
 *  \return video mode info string
 */
std::string GetVideoInfoString(const char *indent)
{
  std::string s;
  
  s += indent;
  s += "Renderer:    ";
  AddGLString(s, GL_RENDERER);
  s += "\n";
  s += indent;
  s += "Vendor:      ";
  AddGLString(s, GL_VENDOR);
  s += "\n";
  s += indent;
  s += "GL version:  ";
  AddGLString(s, GL_VERSION);
  s += "\n";
  s += indent;
  s += "RGBA bpp:    ";
  s += itoStr(vidbits.red, ' ', 0);
  s += "/";
  s += itoStr(vidbits.green, ' ', 0);
  s += "/";
  s += itoStr(vidbits.blue, ' ', 0);
  s += "/";
  s += itoStr(vidbits.alpha, ' ', 0);
  s += "\n";
  s += indent;
  s += "Depth bpp:   ";
  s += itoStr(vidbits.depth, ' ', 0);
  s += "\n";
  s += indent;
  s += "Stencil bpp: ";
  s += itoStr(vidbits.stencil, ' ', 0);
  s += "\n";
  
  return s;
}


SDL_Surface* video_setup_colordepth(int& nX, int& nY, int& nFullscreen, int color_depth, int nMultisamples)
{
  SDL_Surface* screen;

  int cbits = (color_depth <= 16) ?  5 :  8;
  int zbits = (color_depth <= 16) ? 16 : 24;
  
  if (nMultisamples > 0)
  {
    cbits =  8;
    if (zbits == 24)
      zbits = 32;
  }
  
  /* setup GL attributes */
  SDL_GL_SetAttribute(SDL_GL_RED_SIZE, cbits);
  SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, cbits);
  SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, cbits);
//  SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, cbits);
  SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE,1);
  SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, zbits);
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
  
  if (nMultisamples > 0)
  {
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, nMultisamples);
    glEnable(GL_MULTISAMPLE);
  
    // http://adrianboeing.blogspot.de/2010/01/antialiasing-multisampling-opengl-with.html
    glHint(GL_LINE_SMOOTH_HINT, GL_FASTEST);
    glHint(GL_POLYGON_SMOOTH_HINT, GL_FASTEST);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POLYGON_SMOOTH);
  }
  else
  {
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 0);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 0);
    glDisable(GL_MULTISAMPLE);
  }
  
  // Load defaults?
  if (nX < 100 || nY < 100)
  {
    SimpleXMLTransfer* item;
    nFullscreen = cfgfile->getInt("video.fullscreen.fUse", 0);
    if (nFullscreen)
    {
      item = cfgfile->getChild("video.resolution.fullscreen", true);
      nX = item->getInt("x", 0);// "0" : auto resolution
      nY = item->getInt("y", 0);// "0" : auto resolution
    }
    else
    {
      item = cfgfile->getChild("video.resolution.window", true);
      nX = item->getInt("x", 800);
      nY = item->getInt("y", 600);
    }

    printf("Loading default videomode from config...\n");
  }

  /* setup SDL fullscreen flag */
  if (nFullscreen)
  {
    SDL_video_flags |= SDL_FULLSCREEN;
    SDL_video_flags &= ~SDL_RESIZABLE;
  }
 else
  {
    SDL_video_flags &= ~SDL_FULLSCREEN;
    SDL_video_flags |= SDL_RESIZABLE;
  }
// try to setup...

  screen = SDL_SetVideoMode(nX, nY, color_depth, SDL_video_flags);

  // If that one failed, take window mode from config file:
  if (!screen)
  {
    printf("Failed to setup videomode %ix%i, fullscreen=%i\n", nX, nY, nFullscreen);
    nX = cfgfile->getInt("video.resolution.window.x", 800);
    nY = cfgfile->getInt("video.resolution.window.y", 600);
    nFullscreen = 0;
    SDL_video_flags &= ~SDL_FULLSCREEN;
    screen = SDL_SetVideoMode(nX, nY, color_depth, SDL_video_flags);
  }

  // Did that one fail, too?
  if (!screen)
  {
    printf("Failed to setup videomode %ix%i, fullscreen=%i\n", nX, nY, nFullscreen);
    SDL_video_flags &= ~SDL_FULLSCREEN;
    if (nFullscreen)
    {
      SDL_video_flags |= SDL_FULLSCREEN;
    }
    nX = 640;
    nY = 480;
    screen = SDL_SetVideoMode(nX, nY, color_depth, SDL_video_flags);
  }

  return(screen);
}

int setupScreen(int nX, int nY, int nFullscreen)
{
  int resolution_auto = 0;
  if((nX==0)&&(nY==0)&&(nFullscreen==0))//intialization
  {
    const SDL_VideoInfo* vi = SDL_GetVideoInfo();
    screen_xsize = vi->current_w;
    screen_ysize = vi->current_h;
    printf("Screen resolution : %d x %d \n",screen_xsize, screen_ysize);
  }
  int color_depth = cfgfile->getInt("video.color_depth", 24);
  int nMultisamples = cfgfile->getInt("video.multisamples", 1); // anti aliasing: 0 (off), 1, 2, 4, 8, 16
    
  SDL_video_flags = SDL_OPENGL;
#if defined linux || defined WIN32
  SDL_video_flags |= SDL_RESIZABLE;
#endif
  if((nX==0)&&(nY==0)&&(nFullscreen==1))//resolution auto
  {
    nX = screen_xsize;
    nY = screen_ysize;
    resolution_auto = 1;
  }
  SDL_Surface* screen = video_setup_colordepth(nX, nY, nFullscreen, color_depth, nMultisamples);

  if (!screen)
  {
    color_depth = 32;
    screen = video_setup_colordepth(nX, nY, nFullscreen, color_depth, nMultisamples);
  }
  if (!screen)
  {
    color_depth = 16;
    screen = video_setup_colordepth(nX, nY, nFullscreen, color_depth, nMultisamples);
  }
  if (!screen && nMultisamples > 0)
  {
    color_depth = 32;
    nMultisamples = 0;
    screen = video_setup_colordepth(nX, nY, nFullscreen, color_depth, nMultisamples);
  }
  if (!screen && nMultisamples > 0)
  {
    color_depth = 16;
    nMultisamples = 0;
    screen = video_setup_colordepth(nX, nY, nFullscreen, color_depth, nMultisamples);
  }
  if (!screen)
  {
    color_depth = 8;
    screen = video_setup_colordepth(nX, nY, nFullscreen, color_depth, nMultisamples);
  }

  if (!screen)
  {
    printf("Failed to setup videomode %ix%i, fullscreen=%i\n", nX, nY, nFullscreen);
    fprintf(stderr, "Unable to setup any video mode. Exiting.\n");
    crrc_exit(CRRC_EXIT_FAILURE,
    #ifdef WIN32
        "Unable to setup any video mode.\nSee stdout.txt for more information."
    #else
        "Unable to setup any video mode.\nSee stdout for more information."
    #endif
      );
  }

  // Write back to config.
  if (resolution_auto ) 
  {
    nX = 0;
    nY = 0;
  }
  if (nFullscreen)
  {
    cfgfile->setAttributeOverwrite("video.fullscreen.fUse", "1");
    cfgfile->setAttributeOverwrite("video.resolution.fullscreen.x", nX);
    cfgfile->setAttributeOverwrite("video.resolution.fullscreen.y", nY);
  }
  else
  {
    cfgfile->setAttributeOverwrite("video.fullscreen.fUse", "0");
    cfgfile->setAttributeOverwrite("video.resolution.window.x", nX);
    cfgfile->setAttributeOverwrite("video.resolution.window.y", nY);
  }
  cfgfile->setAttributeOverwrite("video.color_depth", color_depth);
  cfgfile->setAttributeOverwrite("video.multisamples", nMultisamples);
  
  window_xsize = screen->w;
  window_ysize = screen->h;
  
  // Store the received buffer depths for fast access
  SDL_GL_GetAttribute(SDL_GL_RED_SIZE, &(vidbits.red));
  SDL_GL_GetAttribute(SDL_GL_GREEN_SIZE, &(vidbits.green));
  SDL_GL_GetAttribute(SDL_GL_BLUE_SIZE, &(vidbits.blue));
  SDL_GL_GetAttribute(SDL_GL_ALPHA_SIZE, &(vidbits.alpha));
  SDL_GL_GetAttribute(SDL_GL_DEPTH_SIZE, &(vidbits.depth));
  SDL_GL_GetAttribute(SDL_GL_STENCIL_SIZE, &(vidbits.stencil));
  
  std::string s = GetVideoInfoString("  ");
  printf("Using the following rendering mode:\n%s", s.c_str());
  printf("  ");
  Video::dumpGLStackInfo(stdout);

  return(0);
}

/*****************************************************************************/
void setWindowTitleString()
{
  SDL_WM_SetCaption("CRRCSim: Charles River RC Flight Simulator", "CRRCSim");
}

/*****************************************************************************/
void reshape(int w, int h)
{
  window_xsize = w;
  window_ysize = h;

  // save current screen resolution
  SimpleXMLTransfer* res = cfgfile->getChild("video.resolution");
  if (cfgfile->getInt("video.fullscreen.fUse"))
    res = res->getChild("fullscreen");
  else
    res = res->getChild("window");
  res->setAttributeOverwrite("x", window_xsize);
  res->setAttributeOverwrite("y", window_ysize);
}


/** \brief Resize the application window
 *
 *  Currently this only works for Linux. On other platforms
 *  you have to restart CRRCsim to activate the new resolution.
 *  This is a limitation of SDL on non-Linux platforms.
 *
 *  \param w new window width
 *  \param h new window height
 *
 *  \todo Find a window resize solution for non-Linux platforms.
 */
void resize_window(int w, int h)
{
#if defined linux || defined WIN32
  SDL_Surface *screen;
  if(!(SDL_video_flags&SDL_FULLSCREEN)) // for WinXP
  {
    //SDL_video_flags &= ~SDL_FULLSCREEN;
    screen = SDL_SetVideoMode(w, h,
                              cfgfile->getInt("video.color_depth"),
                              SDL_video_flags);
    if (screen)
      reshape(screen->w, screen->h);
  }
#else
  // until we find a solution, just ignore the event
  // --------------------------------------------------
  // This solution does not work on Win98:
  // rebuild_video_context(event.resize.w, event.resize.h)
  // --------------------------------------------------
#endif
}


void cleanup()
{
  // Skip cleanup in headless mode (console and sky weren't initialized)
  if (cfgfile->getInt("video.enabled", 1) == 0)
    return;
    
  delete console;
  cleanup_sky();
}


void read_config(SimpleXMLTransfer* cf)
{
  flSmartCam = cf->getDouble("video.camera.smart", 0);
  flSloppyCam = cf->getDouble("video.camera.sloppy", 0);
}


/** drawSolidCube() is a modified version of drawBox
 *  from GLUT 3.7, file glut_shapes.c. It serves as
 *  a replacement for glutSolidCube().
 *  Please keep the following license information.
 */

/* Copyright (c) Mark J. Kilgard, 1994, 1997. */

/**
(c) Copyright 1993, Silicon Graphics, Inc.

ALL RIGHTS RESERVED

Permission to use, copy, modify, and distribute this software
for any purpose and without fee is hereby granted, provided
that the above copyright notice appear in all copies and that
both the copyright notice and this permission notice appear in
supporting documentation, and that the name of Silicon
Graphics, Inc. not be used in advertising or publicity
pertaining to distribution of the software without specific,
written prior permission.
*/
void drawSolidCube(GLfloat size)
{
  static GLfloat n[6][3] =
  {
    {-1.0, 0.0, 0.0},
    {0.0, 1.0, 0.0},
    {1.0, 0.0, 0.0},
    {0.0, -1.0, 0.0},
    {0.0, 0.0, 1.0},
    {0.0, 0.0, -1.0}
  };
  static GLint faces[6][4] =
  {
    {0, 1, 2, 3},
    {3, 2, 6, 7},
    {7, 6, 5, 4},
    {4, 5, 1, 0},
    {5, 6, 2, 1},
    {7, 4, 0, 3}
  };
  GLfloat v[8][3];
  GLint i;

  v[0][0] = v[1][0] = v[2][0] = v[3][0] = -size / 2;
  v[4][0] = v[5][0] = v[6][0] = v[7][0] = size / 2;
  v[0][1] = v[1][1] = v[4][1] = v[5][1] = -size / 2;
  v[2][1] = v[3][1] = v[6][1] = v[7][1] = size / 2;
  v[0][2] = v[3][2] = v[4][2] = v[7][2] = -size / 2;
  v[1][2] = v[2][2] = v[5][2] = v[6][2] = size / 2;

  for (i = 5; i >= 0; i--) {
    glBegin(GL_QUADS);
    glNormal3fv(&n[i][0]);
    glVertex3fv(&v[faces[i][0]][0]);
    glVertex3fv(&v[faces[i][1]][0]);
    glVertex3fv(&v[faces[i][2]][0]);
    glVertex3fv(&v[faces[i][3]][0]);
    glEnd();
  }
}

/* end modified GLUT code */


void InitSmartCamera()
{
  // Skip smart camera initialization in headless mode
  if (cfgfile->getInt("video.enabled", 1) == 0)
    return;
    
  flSmartCam_time = 0.;

  CRRCMath::Vector3 plane_pos = FDM2Graphics(Global::aircraft->getPos());
  CRRCMath::Vector3 plane_dir = plane_pos - player_pos;
  CRRCMath::Vector3 plane_dir_FDM = Graphics2FDM(plane_dir);
  flSmartCam_center_w = atan2(plane_dir_FDM.r[1], plane_dir_FDM.r[0]); 
  
  double dist = sqrt(plane_dir_FDM.r[0]*plane_dir_FDM.r[0] + plane_dir_FDM.r[1]*plane_dir_FDM.r[1]) + 1.;
  flSmartCam_center_h = atan(plane_dir_FDM.r[2]/dist);
  
  flSmartCam_var_w = 0.;
  flSmartCam_var_h = 0.;
}


void UpdateCamera(float flDeltaT)
{
  // Skip camera updates in headless mode
  if (cfgfile->getInt("video.enabled", 1) == 0)
    return;
    
  double fieldOfViewRad = M_PI/180.0*zoom_get();
  
  double pi_two  = 2.*M_PI;
  double pi_half = 0.5*M_PI;
  CRRCMath::Vector3 plane_pos = FDM2Graphics(Global::aircraft->getPos());
  CRRCMath::Vector3 plane_dir = plane_pos - player_pos;
  CRRCMath::Vector3 plane_dir_FDM = Graphics2FDM(plane_dir);
  double phi_w = atan2(plane_dir_FDM.r[1], plane_dir_FDM.r[0]);  
  double dist = sqrt(plane_dir_FDM.r[0]*plane_dir_FDM.r[0] + plane_dir_FDM.r[1]*plane_dir_FDM.r[1]) + 1.;
  double phi_h = atan(plane_dir_FDM.r[2]/dist);

  if (!Global::testmode)
  {
    // update smart camera automatic center/range definition
    flSmartCam_time += flDeltaT;
    flSmartCam_time = flSmartCam_time > SMARTCAM_TIME ? SMARTCAM_TIME : flSmartCam_time;
    if (flSmartCam_time > 0.)
    {
      double w = flDeltaT/flSmartCam_time;
      double w1 = 1. - w;

      double dphi;

      dphi = phi_w - flSmartCam_center_w; 
      if (fabs(dphi) > M_PI)
        dphi = dphi > 0. ? dphi - pi_two : dphi + pi_two;
      flSmartCam_center_w += w*dphi;
      if (fabs(flSmartCam_center_w) > M_PI)
        flSmartCam_center_w = flSmartCam_center_w > 0. ? flSmartCam_center_w - pi_two : flSmartCam_center_w + pi_two;
      flSmartCam_var_w = w1*flSmartCam_var_w + w*dphi*dphi;
        
      dphi = phi_h - flSmartCam_center_h;
      flSmartCam_center_h += w*dphi;
      if (fabs(flSmartCam_center_h) > pi_half)
        flSmartCam_center_h = flSmartCam_center_h > 0. ? pi_half : -pi_half;
      flSmartCam_var_h = w1*flSmartCam_var_h + w*dphi*dphi;
    }
  }
  
  if (flSmartCam > 0 && !Global::testmode) // smart camera option active
  {  
    float phi_w0, phi_h0, range_w, range_h;

    // try to get game-specific parameters to drive smart camera
    // If not available use automatic center/range definition
    if (!Global::gameHandler->GetSmartCameraPar(phi_w0, phi_h0, range_w, range_h))
    {
      phi_w0 = flSmartCam_center_w;
      phi_h0 = flSmartCam_center_h;
      range_w = SMARTCAM_RANGE_FAC*sqrt(flSmartCam_var_w);
      range_h = SMARTCAM_RANGE_FAC*sqrt(flSmartCam_var_h);
    }
    
    // always use full movement range in height
    // but only used specified range in width
    CRRCMath::Vector3 look_dir_FDM(plane_dir_FDM);
    double dphimax_w0 = 0.5*fieldOfViewRad*SMARTCAM_MAX_FRACTION*flSmartCam;
    double dphimax_h0 = 0.5*fieldOfViewRad*SMARTCAM_MAX_FRACTION*window_ysize/window_xsize;
    range_w = range_w > dphimax_w0 ? range_w : dphimax_w0;
    range_h = range_h > dphimax_h0 ? range_h : dphimax_h0;
    
    // set look_dir orientation
    {
      double dphi = phi_w - phi_w0; 
      if (fabs(dphi) > M_PI)
        dphi = dphi > 0. ? dphi - pi_two : dphi + pi_two;
      double sign = dphi >= 0. ? 1. : -1.;
      if (fabs(dphi) < range_w)
        dphi = sign*dphimax_w0*sin(pi_half*fabs(dphi)/range_w);
      else
        dphi = sign*dphimax_w0*sin(pi_half*(M_PI - fabs(dphi))/(M_PI - range_w));

      // rotate plane_dir_FDM towards look_dir_FDM along Z axis 
      double cc = cos(dphi);
      double ss = sin(dphi);
      look_dir_FDM.r[0] =  cc*plane_dir_FDM.r[0] + ss*plane_dir_FDM.r[1];
      look_dir_FDM.r[1] = -ss*plane_dir_FDM.r[0] + cc*plane_dir_FDM.r[1];
    }

    // set look_dir elevation
    {
      double dphi = phi_h - phi_h0;    
      double sign = dphi >= 0. ? 1. : -1.;
      if (fabs(dphi) < range_h)
        dphi = sign*dphimax_h0*sin(pi_half*fabs(dphi)/range_h);
      else
        dphi = sign*dphimax_h0;
      look_dir_FDM.r[2] = dist*tan(phi_h - dphi);
    }
    
    // set looking position
    looking_pos = player_pos + FDM2Graphics(look_dir_FDM);
  }
  else
  {
    double  phimax   = flSloppyCam*fieldOfViewRad;
    double  max      = cos(phimax)*cos(phimax);
    
    CRRCMath::Vector3 look_dir  = looking_pos - player_pos;
    CRRCMath::Vector3 plane_dir = plane_pos - player_pos;
    
    if (plane_dir.angle_cos_sqr(look_dir) < max)
    {
      // Adjust the length of look_dir so that plane_dir, look_dir and
      // (look_dir - plane_dir) form a right angle triangle.
      double k = (plane_dir.inner(plane_dir)) / (plane_dir.inner(look_dir));
      look_dir = look_dir * k;
      // Calculate the difference vector...
      CRRCMath::Vector3 diff_dir = look_dir - plane_dir;
      double ddl = diff_dir.length();
      if (ddl > 0.001)
      {
        // ...and adjust its length (the right angle triangle remains!) so that
        // the angle between look_dir and plane_dir equals phimax.
        double tan_phi = fabs(tan(acos(sqrt(max))));
        look_dir = plane_dir + diff_dir*(plane_dir.length()*tan_phi/ddl);
        looking_pos = look_dir + player_pos;
      }
    }
    else
    {
      // slowly follow model
      CRRCMath::Vector3 diff_pos = plane_pos - looking_pos;
      looking_pos = looking_pos + diff_pos*(0.02*flDeltaT);
    }
  }
}


/**
 * Get the size of the current window.
 * \param x   x size in pixels will be written to this variable
 * \param y   y size in pixels will be written to this variable
 */
void getWindowSize(int& x, int& y)
{
  x = window_xsize;
  y = window_ysize;
}

/**
 * Read the "smart cam" setting from mod_video
 */
float getSmartCam()
{
  return flSmartCam;
}

/**
 * Write the "smart cam" setting
 * \param flValue  New value for smart cam
 */
void setSmartCam(float flValue)
{
  flSmartCam = flValue;
}

/**
 * Read the "sloppy cam" setting from mod_video
 */
float getSloppyCam()
{
  return flSloppyCam;
}

/**
 * Write the "sloppy cam" setting
 * \param flValue  New value for sloppy cam
 */
void setSloppyCam(float flValue)
{
  flSloppyCam = flValue;
}


/**
 * Initialize the console overlay
 */
void initConsole()
{
  console = new GlConsole(400, 125, 15, 15);
  if (console == NULL)
  {
    crrc_exit(CRRC_EXIT_FAILURE, "Unable to initialize console.");
  }
  console->setAutoHide(4000, 1000);
  LOG(_("CRRCsim successfully started!"));
  LOG(_("Press <ESC> to show the setup menu."));
}

} // end of namespace Video::
