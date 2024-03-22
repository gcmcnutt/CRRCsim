/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2007 Tom Willis (original author)
 * Copyright (C) 2007 Jan Reucker
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
  

/*
 *  puaGLPreview.cpp
 *
 *  pui widget to display an OpenGL preview.
 *  Copyright (C) 2007 twillis
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2
 *  as published by the Free Software Foundation. or
 *  (at your option) any later version. 
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include "puaGLPreview.h"

// graphics parameters for the preview
//
// TODO these should be defaults of ivs that can be set dynamically.  too lazy to write the accessors right
// TODO now (and the accessors should be part of the view behavior specialization mentioned in the .h file
// TODO anyway).
//
#define GLPREVIEW_INSET  (6)          // pixels to inset active preview region from abox

#define GLPREVIEW_BKGND_R (0.4863)    // background color, a calming sky blue (thanks pixie.app!)
#define GLPREVIEW_BKGND_G (0.5608)
#define GLPREVIEW_BKGND_B (0.7373)

#define GLPREVIEW_HORIZ_FOV (60.0)    // horizontal field of view for preview (deg)

#define GLPREVIEW_BS_MULT   (1.25)    // better living through gratuitous fudge factor hackery

#define GLPREVIEW_CAMERA_X (1.50)     // default camera x location
#define GLPREVIEW_CAMERA_P  (40.0)    // default camera pitch (deg)

// computes h, w of active preview region.
//
#define GLPREVIEW_ACTIVE_W   (abox.max[0] - abox.min[0] - GLPREVIEW_INSET)
#define GLPREVIEW_ACTIVE_H   (abox.max[1] - abox.min[1] - GLPREVIEW_INSET)

// ambient light to use.
//
static GLfloat ambientLight[] = { 0.55, 0.55, 0.55, 1.0 };

/**
 *  Construct a puaGLPreview
 */
puaGLPreview::puaGLPreview(int minx, int miny, int maxx, int maxy)
    : puFrame(minx, miny, maxx, maxy),
    context(NULL), scene(NULL), geometry(NULL), transInitial(NULL), transGeometry(NULL),
    frame(0)
{
  // create an ssg context for our drawing and set it up.  the %#$)* constructor always sets the new object to the current
  // context (grrrr), so we need to do a push/pop to make sure we don't disturb things.
  //
  ssgContext *curContext = ssgGetCurrentContext();
  context = new ssgContext();
  curContext->makeCurrent();

  // setup the scene graph.  we start with a root, followed by the initial transform to change coordinate systems, followed
  // by the transform to adjust the geometry.
  //
  scene = new ssgRoot();
  transInitial = new ssgTransform();
  transInitial->setName("transInitial");
  transGeometry = new ssgTransform();
  transGeometry->setName("transGeometry");
  scene->addKid(transGeometry);
  transGeometry->addKid(transInitial);

  updateViewParameters();
}


/**
 *  Destroy the puaGLPreview
 */
puaGLPreview::~puaGLPreview()
{
  delete context;

  if (geometry != NULL)
  {
    transInitial->removeKid(geometry);
  }
  transGeometry->removeKid(transInitial);
  scene->removeKid(transGeometry);
  delete scene;
}


/**
 *  Updates the view parameters (field of view, etc) of our ssg context to match a change in the size of the puFrame's
 *  abox.  This method preserves the current ssg drawing context, only the widget-internal context is changed.
 *
 *  \todo Why is this method necessary? The draw() method will always override the FOV specified here.
 */
void puaGLPreview::updateViewParameters()
{
  context->setFOV(GLPREVIEW_HORIZ_FOV, GLPREVIEW_HORIZ_FOV * (GLPREVIEW_ACTIVE_W / GLPREVIEW_ACTIVE_H));
}


/**
 *  Changes the size of the abox for the widget.  When this happens, in addition to the usual pui stuff, we need
 *  to make sure to match the view paramters to the new size.
 */
void puaGLPreview::setSize(int w, int h)
{
  puFrame::setSize(w, h);
  updateViewParameters();
}


/**
 *  Draw the GL preview.
 *
 *  Use the inherited draw() method to take care of doing the basics of the preview (ie, the normal parts of a
 *  puFrame like the label, legend, and box. The geometry scene graph that makes up the content we are previewing
 *  is rendered via ssgCullAndDraw().  This method uses the setupCamera() and setupGeometry() methods to set the
 *  transforms and such for the camera/geometry (to make things spin and so forth, wheeee!).
 *
 *  To effectively change the perspective in the preview, this method must be invoked at a regular rate.
 *
 *  This method should preserve ssg drawing context and the opengl projection matrix.  It will trash the opengl
 *  depth buffer for what that's worth.
 */
void puaGLPreview::draw(int dx, int dy)
{
  if ( !visible || ( window != puGetWindow () ) )
    return ;

  // first, let pui do its thing for the draw.  this will get us the frame and any text drawn correctly.
  //
  puFrame::draw ( dx, dy );

  if (geometry == NULL)
    return;         // get ouf of dodge if we have no geometry

  ssgContext *prevContext = ssgGetCurrentContext();    // current ssg context so we can avoid trampling
  int w = GLPREVIEW_ACTIVE_W;         // dimensions of preview area
  int h = GLPREVIEW_ACTIVE_H;

  // update the camera and geometry position as needed for the preview (subclasses will over-ride these to do the
  // right thing).  advance the running "frame" count while we are at it.
  //
  setupCamera(frame);
  setupGeometry(frame++);

  // prepare to draw the preview.  we save the opengl attributes and projection matrix here so that we can restore
  // them later to avoid confusing others.  the opengl viewport is changed to cover just the preview area.
  //
  glPushAttrib(GL_ALL_ATTRIB_BITS);

  // Save the current projection matrix.
  // Instead of pushing the current projection matrix on the stack, create a copy of the current
  // matrix and restore it later, just in case the current system's projection matrix stack depth is only 2.
  // Remember, PUI will always push the projection matrix when it starts rendering, so the stack may 
  // be already full (OpenGL guarantees only 2 levels on the projection matrix stack).
  glMatrixMode(GL_PROJECTION);
  GLfloat current_proj_matrix[16];
  glGetFloatv(GL_PROJECTION_MATRIX, current_proj_matrix);

  glViewport(abox.min[0] + dx + (GLPREVIEW_INSET / 2), abox.min[1] + dy + (GLPREVIEW_INSET / 2), w, h);

  // fill the preview area with a background color with a glRect and ortho projection.  make sure to disable depth
  // tests here to avoid obscuring what comes next (remember that glRects are at depth 0 by default).
  //
  /// \todo In addition to disabling the depth test it would maybe make sense to disable depth buffer writes
  ///       to avoid that the model "collides" with the glRect. This would maybe eliminate the need for
  ///       clearing the depth buffer before drawing the model, although, in that case, the model could
  ///       interfere with the underlying scenery.
  glMatrixMode(GL_PROJECTION);    // we should still be in projection mode, but just in case...
  glLoadIdentity();
  glOrtho(0, w - 1, 0, h - 1, -1, 1);
  glDisable(GL_DEPTH_TEST);

  glColor3f(GLPREVIEW_BKGND_R, GLPREVIEW_BKGND_G, GLPREVIEW_BKGND_B);
  glRecti(0, 0, w - 1, h - 1);

  // render the geometry in the preview area.  in this case, enable depth testing so that we can appropriately
  // draw the scene along with some ambient light.  note that the rendering takes place in our own ssg context (we
  // switch back to the previous context after we are done).
  //
  glEnable(GL_DEPTH_TEST);
  glClear(GL_DEPTH_BUFFER_BIT);
  glEnable(GL_LIGHTING);
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambientLight);

  context->makeCurrent();
  context->setFOV(45, 45);
  ssgCullAndDraw(scene);
  prevContext->makeCurrent();

  // restore the previous opengl projection matrix and opengl attributes before returning.
  //
  glMatrixMode(GL_PROJECTION);
  glLoadMatrixf(current_proj_matrix);

  glPopAttrib();
}


/**
 *  Load the geometry and textures for the object to display.  returns false on success, true on failure.
 */
bool puaGLPreview::loadGeometry( const char * modelPath, const char * texturePath )
{
  if (geometry != NULL)
  {     // free up any previous model
    transInitial->removeKid(geometry);
    geometry = NULL;
  }

  ssgTexturePath(texturePath);    // load the model and textures via ssg
  geometry = ssgLoad(modelPath);

  // if the geometry load fails, we will setup the legend to indicate "huston, we have a problem."  otherwise, add
  // the geometry to the scene graph (as a child of the initial transform).  the initial transform is set to
  // translate the geometry such that the center of its bounding sphere is at (0, 0, 0).
  //
  if (geometry == NULL)
  {
    setLegend("Cannot load geometry.");  // "huston, we have a problem", use legend to let the luser know
  }
  else
  {
    geometry->setName("geometry");
    transInitial->addKid(geometry);

    const SGfloat *center = geometry->getBSphere()->getCenter();
    sgMat4 xform = {  { 1.0,  0.0,  0.0,  0.0 },
                      { 0.0,  1.0,  0.0,  0.0 },
                      { 0.0,  0.0,  1.0,  0.0 },
                      { -center[0], -center[1], -center[2], 1.0 } };
    transInitial->setTransform(xform);
  }

  return (geometry != NULL) ? false : true;
}




// -------- puaGLPreviewGR ------------------------------------------------------------------------

// in this version of puaGLPreview, the geometry rotates while the camera remains stationary.  this is used for previews
// where the view is of an "object" rather than from within a "scene".

// UNDONE eventually break these out, for now, just let puaGLPreview be puaGLPreviewGR

/**
 *  Setup the camera for the preview.  In this class, the geometry rotates while the camera remains stationary.  The camera
 *  is above and behind the geometry (which is at the origin) and looks at the origin.  This method updates the camera
 *  position in our context and assumes that the geometry has been loaded.
 *
 *  the frame parameter is unused.
 */
void puaGLPreview::setupCamera(int frame)
{
  sgVec3   position;    // position of camera
  sgVec3   center;     // vector to center of view

  assert(geometry != NULL);

  // in this preview, the geometry is at (0, 0, 0).  the camera looks back at the origin and is located at (0, -y, z) where
  // y is a distance far eoungh away to keep the geometry's bounding sphere (mostly) within the field of view.
  //
  center[0] = 0;
  center[1] = 0;
  center[2] = 0;

  position[0] = GLPREVIEW_CAMERA_X;
  position[1] = -scene->getBSphere()->getRadius() -
                (GLPREVIEW_BS_MULT * scene->getBSphere()->getRadius()) / sgTan(GLPREVIEW_HORIZ_FOV);
  position[2] = position[0] * sgTan(GLPREVIEW_CAMERA_P);

  // update our context to have the camera where we want it.
  //
  context->setCameraLookAt(position, center);
}

/**
 *  Setup the geometry tranform for the preview.  In this class, the geometry rotates while the camera remains stationary.
 *  This method updates the geometry transform based on the frame number (used to specify rotation about x).
 */
void puaGLPreview::setupGeometry(int frame)
{
  sgCoord   coord;     // translation and rotation

  coord.xyz[0] = 0;
  coord.xyz[1] = 0;
  coord.xyz[2] = 0;

  coord.hpr[0] = frame;
  coord.hpr[1] = 0;
  coord.hpr[2] = 0;

  transGeometry->setTransform(&coord);
}




// -------- puaGLPreviewCR ------------------------------------------------------------------------

// in this version of puaGLPreview, the camera rotates while the geometry remains stationary.  this is used for previews
// where the view is of a "scene" rather than from within a "object".

// UNDONE
