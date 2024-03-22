/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 *  Copyright (C) 2010 Joel Lienard (original author)
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
 * crrc_viewwind.cpp
 * classe  CGUIViewWindDialog
 * visualisation of slope and wind profils, for debug purpose
 */
 
#include "../i18n.h"
#include "crrc_gui_viewwind.h"

#include <stdio.h>
#include <stdlib.h>

#include "util.h"
#include "../crrc_main.h"
#include "../global.h"
#include "../mod_misc/lib_conversions.h"
#include "../mod_misc/SimpleXMLTransfer.h"
#include "../mod_windfield/windfield.h"
#include "../mod_landscape/crrc_scenery.h"

static void CGUIViewWindDialogCallback(puObject *obj);
static void CGUIViewWindSliderCallback(puObject *obj);
static void CGUIViewWindButtonBoxCallback(puObject *obj);
static void CGUIViewWindRenderCallback(class puObject *, int dx, int dy, void *);

char** WindStringsGUI;
int nWindStringsGUI;
char** ColorStringsGUI;
int nColorStringsGUI;

#define BUTTON_BOX_HEIGHT   (2*DLG_DEF_SPACE+DLG_DEF_BUTTON_HEIGHT)
#define SLIDER_W            300
#define SLIDER_H            DLG_DEF_BUTTON_HEIGHT
#define COMBO_W             230
#define COMBO_H             DLG_DEF_BUTTON_HEIGHT
#define LABEL_W             150
#define NUM_W               60
#define BUT_BOX_W           200
#define BUT_BOX_H           (DLG_DEF_SPACE+2*DLG_DEF_BUTTON_HEIGHT)
#define GRAPHE_W            900
#define GRAPHE_H            450
#define SLIDER_FRACTION     0.05

CGUIViewWindDialog::CGUIViewWindDialog()
  : CRRCDialog()
{ 
  std::vector<std::string> lWindStringsGUI(2);
  lWindStringsGUI[0]  = _("Mode 1 (geom based)");
  lWindStringsGUI[1]  = _("Mode 2 (CFD based)");
  WindStringsGUI = T_GUI_Util::loadnames(lWindStringsGUI, nWindStringsGUI);

  std::vector<std::string> lColorStringsGUI(2);
  lColorStringsGUI[0]  = _("In-plane velocity");
  lColorStringsGUI[1]  = _("Vertical velocity");
  ColorStringsGUI = T_GUI_Util::loadnames(lColorStringsGUI, nColorStringsGUI);

  // Create widgets
  //
  
  //inital values
  position = 0;
  direction = cfg->wind->getDirection();
  width = 1000;
  hoffs = 0;
  voffs = 0;
  wind_mode = Global::wind_mode;
  color_mode = 2;

  int ypos = BUTTON_BOX_HEIGHT;
  int xpos = LABEL_W + DLG_DEF_SPACE + SLIDER_W;

  //slider voffs
  slider_voffs = new crrcSlider(LABEL_W + DLG_DEF_SPACE, ypos,
                                LABEL_W + DLG_DEF_SPACE + SLIDER_W,  ypos + SLIDER_H,
                                NUM_W);
  ypos += (SLIDER_H + DLG_DEF_SPACE);
  slider_voffs->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_voffs->setLabel(_("Vert. offset"));
  slider_voffs->setSliderFraction(SLIDER_FRACTION);
  slider_voffs->setMinValue(-2000);
  slider_voffs->setMaxValue(+2000);
  slider_voffs->setStepSize(50);
  slider_voffs->setValue(voffs);
  slider_voffs->setCallback(CGUIViewWindSliderCallback);
  slider_voffs->setUserData(this);

  //slider hoffs
  slider_hoffs = new crrcSlider(LABEL_W + DLG_DEF_SPACE, ypos,
                                LABEL_W + DLG_DEF_SPACE + SLIDER_W,  ypos + SLIDER_H,
                                NUM_W);
  ypos += (SLIDER_H + DLG_DEF_SPACE);
  slider_hoffs->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_hoffs->setLabel(_("Horiz. offset"));
  slider_hoffs->setSliderFraction(SLIDER_FRACTION);
  slider_hoffs->setMinValue(-5000);
  slider_hoffs->setMaxValue(+5000);
  slider_hoffs->setStepSize(100);
  slider_hoffs->setValue(hoffs);
  slider_hoffs->setCallback(CGUIViewWindSliderCallback);
  slider_hoffs->setUserData(this);

  //slider width
  slider_width = new crrcSlider(LABEL_W + DLG_DEF_SPACE, ypos,
                                LABEL_W + DLG_DEF_SPACE + SLIDER_W,  ypos + SLIDER_H,
                                NUM_W);
  ypos += (SLIDER_H + DLG_DEF_SPACE);
  slider_width->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_width->setLabel(_("Zoom width"));
  slider_width->setSliderFraction(SLIDER_FRACTION);
  slider_width->setMinValue(50);
  slider_width->setMaxValue(+30000);
  slider_width->setStepSize(50);
  slider_width->setValue(width);
  slider_width->setCallback(CGUIViewWindSliderCallback);
  slider_width->setUserData(this);

  //slider position
  slider_position = new crrcSlider(LABEL_W + DLG_DEF_SPACE, ypos,
                                   LABEL_W + DLG_DEF_SPACE + SLIDER_W,  ypos + SLIDER_H,
                                   NUM_W);
  ypos += (SLIDER_H + DLG_DEF_SPACE);
  slider_position->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_position->setLabel(_("Position (left<->right)"));
  slider_position->setSliderFraction(SLIDER_FRACTION);
  slider_position->setMinValue(-5000);
  slider_position->setMaxValue(+5000);
  slider_position->setStepSize(10);
  slider_position->setValue(position);
  slider_position->setCallback(CGUIViewWindSliderCallback);
  slider_position->setUserData(this);
  
  //slider direction
  slider_direction = new crrcSlider(LABEL_W + DLG_DEF_SPACE, ypos,
                                    LABEL_W + DLG_DEF_SPACE + SLIDER_W, ypos + SLIDER_H,
                                    NUM_W);
  ypos += (SLIDER_H + DLG_DEF_SPACE);
  slider_direction->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_direction->setLabel(_("Direction (degrees)"));
  slider_direction->setSliderFraction(SLIDER_FRACTION);
  slider_direction->setMinValue(0);
  slider_direction->setMaxValue(360);
  slider_direction->setStepSize(1);
  slider_direction->setValue(direction);
  slider_direction->setCallback(CGUIViewWindSliderCallback);
  slider_direction->setUserData(this);

  //wind vectors color mode
  buttonbox_color = new puButtonBox(xpos + LABEL_W + DLG_DEF_SPACE,
                                    ypos - 2*BUT_BOX_H - 2*DLG_DEF_SPACE,
                                    xpos + LABEL_W + DLG_DEF_SPACE + BUT_BOX_W,
                                    ypos - BUT_BOX_H - 2*DLG_DEF_SPACE,
                                    (char**)ColorStringsGUI, 1);
  buttonbox_color->setLabelPlace(PUPLACE_CENTERED_LEFT);
  buttonbox_color->setLabel(_("Color mode"));
  buttonbox_color->setValue(color_mode-1);
  buttonbox_color->setCallback(CGUIViewWindButtonBoxCallback);
  buttonbox_color->setUserData(this);
  
  //wind computation mode
  buttonbox_wind = new puButtonBox(xpos + LABEL_W + DLG_DEF_SPACE,
                                   ypos - BUT_BOX_H - DLG_DEF_SPACE,
                                   xpos + LABEL_W + DLG_DEF_SPACE + BUT_BOX_W,
                                   ypos - DLG_DEF_SPACE,
                                   (char**)WindStringsGUI, 1);
  xpos += (BUT_BOX_W + DLG_DEF_SPACE);
  buttonbox_wind->setLabelPlace(PUPLACE_CENTERED_LEFT);
  buttonbox_wind->setLabel(_("Wind mode"));
  buttonbox_wind->setValue(wind_mode-1);
  buttonbox_wind->setCallback(CGUIViewWindButtonBoxCallback);
  buttonbox_wind->setUserData(this);
  
  //graphical frame
  graphe = new puFrame(DLG_DEF_SPACE, ypos, 
                       DLG_DEF_SPACE + GRAPHE_W, ypos + GRAPHE_H);
  ypos += (GRAPHE_H + DLG_DEF_SPACE);
  graphe->setRenderCallback(CGUIViewWindRenderCallback, this);

  //size
  int sizew = GRAPHE_W + DLG_DEF_SPACE;
  if (sizew < xpos) sizew = xpos ;

  // finalize the dialog
  close();
  setSize(sizew + DLG_DEF_SPACE, ypos);
  setCallback(CGUIViewWindDialogCallback);
  centerOnScreen();
  reveal();
}

/**
 * Destroy the dialog.
 */
CGUIViewWindDialog::~CGUIViewWindDialog()
{
  T_GUI_Util::freenames(WindStringsGUI, nWindStringsGUI);
  T_GUI_Util::freenames(ColorStringsGUI, nColorStringsGUI);
}

/** The dialog's callback.
 *
 */
void CGUIViewWindDialogCallback(puObject *obj)
{
  if (obj->getIntegerValue() == CRRC_DIALOG_OK)
  {
    // Dialog left by clicking OK
    //CGUIViewWindDialog* dlg   = (CGUIViewWindDialog*)obj;
    //nothing...
  }  
  puDeleteObject(obj);
}

//slider callback
void CGUIViewWindSliderCallback(puObject *obj)
{
  CGUIViewWindDialog* dlg = (CGUIViewWindDialog*)(obj->getUserData());
  dlg->direction = dlg->slider_direction->getIntegerValue();
  dlg->position = dlg->slider_position->getIntegerValue();
  dlg->width = dlg->slider_width->getIntegerValue();
  dlg->hoffs = dlg->slider_hoffs->getIntegerValue();
  dlg->voffs = dlg->slider_voffs->getIntegerValue();
}

//button box callback
void CGUIViewWindButtonBoxCallback(puObject *obj)
{
  CGUIViewWindDialog* dlg = (CGUIViewWindDialog*)(obj->getUserData());
  dlg->wind_mode = dlg->buttonbox_wind->getValue() + 1;
  dlg->color_mode = dlg->buttonbox_color->getValue() + 1;
  
  Global::wind_mode = dlg->wind_mode;
}

void CGUIViewWindRenderCallback(puObject *obj, int dx, int dy,void *data)
{
  CGUIViewWindDialog* dlg = (CGUIViewWindDialog*)data;
  dlg->viewWind2D(obj,dx,dy,dlg->color_mode);
}

void CGUIViewWindDialog::viewWind2D(puObject *obj, int dx, int dy, int color_mode)
{  
#define SCALE_V_MIN   0.5   // lower end of wind speed color scale
#define SCALE_V_MAX   1.5   // higher end of wind speed color scale
#define SCALE_VZ_MIN -0.5   // lower end of vertical wind speed color scale
#define SCALE_VZ_MAX  0.5   // higher end of vertical wind speed color scale

  float flWindVel = cfg->wind->getVelocity(); // freestream wind

  //get draw rectangle
  int x,y,w,h;
  const int marge = 10;
  obj->getPosition( &x, &y );
  obj->getSize( &w, &h );
  x += dx;
  y += dy;
  x += marge;
  y += marge;
  w -= (2*marge);
  h -= (2*marge);
  float h0 = 0.5*(float)h/(float)w;
  //clip
  glScissor(x,y,w,h);
  glEnable(GL_SCISSOR_TEST);
  
  //draw
  glPushMatrix();
  glLoadIdentity();
  glTranslatef(x,y,0);
 
  int n = w/20; //nb of points per trace
  int nc = 30; //nb of trace
  float TRAIT = 0.75/n; //nominal length of wind markers
  float DH = 20*width/1000; //height separation of traces
  float angle = direction/180.0*M_PI;
  float cosi = cos(angle);
  float sinu = sin(angle);
  float ddx = width/n;
  glScalef(w,w,1);
     
  float orig_x = player_pos.r[2] + position*sinu;
  float orig_y = player_pos.r[0] + position*cosi;
  float orig_z = player_pos.r[1];
  float height[n+1];

  //draw slope profile, player position is centered
  //in the screen if hoffs = voffs = 0.
  //NB: GL_POLYGON does not always work fine, since terrain 
  //profile may be concave. Thus only a contour line is drawn
  glEnable(GL_LINE_SMOOTH);
  glLineWidth(3.);
  glColor3f (0.4, 0.4, 0.4); 
  glBegin(GL_LINE_STRIP);
  for(int i=0;i<=n;i++)
  {
    float d = ddx*i - 0.5*width + hoffs;
    float z = Global::scenery->getHeight(orig_x + d*cosi, orig_y + d*sinu);
    if (z > DEEPEST_HELL)
    {
      float x1 = ddx*i/(float)width;
      float y1 = (z - orig_z - voffs)/(float)width + h0;
      glVertex2f(x1,y1);
    }
    height[i] = z;
  }
  glEnd();  

  //draw wind traces
  glLineWidth(2.);
  glBegin(GL_LINES);
  for(int j=1;j<=nc;j++)
    for(int i=0;i<=n;i++)
      if (height[i] > DEEPEST_HELL)
      {
        float d = ddx*i - 0.5*width + hoffs;
        float z = height[i] + DH*j;
        float x_wind, y_wind, z_wind;
        Global::scenery->getWindComponents(orig_x + d*cosi, orig_y + d*sinu, -z,/*z negativ=up*/
                                           &x_wind, &y_wind, &z_wind);
        //horizontal wind
        //float h_wind = sqrt(x_wind*x_wind + y_wind*y_wind);
        //horizontal wind in the analysis plane
        float d_wind = cosi*x_wind + sinu*y_wind;
        //total wind in the analysis plane
        float t_wind = sqrt(d_wind*d_wind + z_wind*z_wind);
        switch (color_mode)
        {
          case 1:
            //set color based on in-plane wind velocity
            setColorFromScale(t_wind/flWindVel, SCALE_V_MIN, SCALE_V_MAX);
            break;
            
          case 2:
            //set color based on vertical wind velocity
            setColorFromScale(-z_wind/flWindVel, SCALE_VZ_MIN, SCALE_VZ_MAX);
            break;
        }
        float x1 = ddx*i/(float)width;
        float y1 = (z - orig_z - voffs)/(float)width + h0;
        arrow2D(x1, y1, d_wind/flWindVel, -z_wind/flWindVel, TRAIT); 
      }
  glEnd();  
  glDisable(GL_LINE_SMOOTH);  

  //draw lines to mark player position
  glLineWidth(2.);
  glLineStipple(2, 0xAAAA);
  glEnable(GL_LINE_STIPPLE);
  glBegin(GL_LINES);
  glColor4f(1., 0., 1., 0.5);  
  // horizontal locator line
  glVertex2f(0., h0 - voffs/(float)width);  
  glVertex2f(1., h0 - voffs/(float)width);  
  // vertical locator line
  glVertex2f(0.5 - hoffs/(float)width, 0.);  
  glVertex2f(0.5 - hoffs/(float)width, 1.);  
  glEnd();  
  glDisable(GL_LINE_STIPPLE);

  glLineWidth(1.);
  glDisable(GL_SCISSOR_TEST);
  glPopMatrix();
}

void CGUIViewWindDialog::setColorFromScale(float val, float min, float max)
{
  //define a blue-green-red color scale from min to max
  
  float r = 0.;
  float g = 0.;
  float b = 0.;
  float range = max - min;
  float mid = 0.5*(max + min);
  float hue = (val - mid)/range;
  hue = hue < -0.5 ? -0.5 : hue > 0.5 ? 0.5 : hue;
  if (hue <= -0.25)
  {
    b = 1.;
    g = (hue + 0.5)/0.25;
  }
  else if (hue <= 0.)
  {
    b = - hue/0.25;
    g = 1.;
  }
  else if (hue <= 0.25)
  {
    g = 1.;
    r = hue/0.25;
  }
  else
  {
    g = - (hue - 0.5)/0.25;
    r = 1.;
  }
  glColor3f(r, g, b);  
}

void CGUIViewWindDialog::arrow2D(float x, float y, float dx, float dy, float lg)
{
  dx *= lg;
  dy *= lg;
  float dx2 = 0.3*dx;
  float dy2 = 0.3*dy;
  glVertex2f(x, y);
  glVertex2f(x + dx, y + dy);
  glVertex2f(x + dx, y + dy);
  glVertex2f(x + dx - dx2 - dy2, y + dy - dy2 + dx2);
  glVertex2f(x + dx, y + dy);
  glVertex2f(x + dx - dx2 + dy2, y + dy - dy2 - dx2);
}
