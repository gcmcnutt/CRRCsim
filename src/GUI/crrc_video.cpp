/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2007, 2008 Jens Wilhelm Wulf (original author)
 * Copyright (C) 2005, 2006, 2008, 2009, 2010 Jan Reucker
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
  

// implementation of class CGUIVideoDialog

#include "../i18n.h"
#include "crrc_video.h"

#include <stdio.h>
#include <stdlib.h>

#include "../crrc_main.h"
#include "../mod_landscape/crrc_scenery.h"
#include "../zoom.h"
#include "../mod_misc/lib_conversions.h"
#include "../CTime.h"
#include "../global.h"
#include "crrc_msgbox.h"
#include "../global_video.h"

static void CGUIVideoCallback(puObject *obj);
static void CGUIVideoSmartCallback(puObject *obj);
static void CGUIVideoSloppyCallback(puObject *obj);
static void CGUIVideoFSCallback(puObject *obj);

#define BUTTON_BOX_HEIGHT   (2*DLG_DEF_SPACE+DLG_DEF_BUTTON_HEIGHT)
#define SLIDER_W            200
#define SLIDER_H            DLG_DEF_BUTTON_HEIGHT
#define COMBO_W             150
#define COMBO_H             DLG_DEF_BUTTON_HEIGHT
#define NUM_W               70
#define LABEL_W             200

#define TOP_WIDGET_Y  (BUTTON_BOX_HEIGHT + 5*SLIDER_H + COMBO_H + 6*DLG_DEF_SPACE + DLG_CHECK_H)
#define AUTOZOOM_MAX  (0.4)
#define FPS_MIN       (20)
#define FPS_MAX       (100)

  /*
  Note (J.L., feb on 2012): the mode "auto resolution" was added for the fullscreen.
  It is coded by "0x0" and shown by "auto".
  It is not thus necessary to leave a list so long of resolution
  */
const int stdResCnt = 9;
const int stdResX[] = {640, 800, 1024, 1024, 1280, 1280, 1280, 1360, 1920, 1920};
const int stdResY[] = {480, 600,  600,  768,  800,  854, 1024, 768,  1080, 1200};

CGUIVideoDialog::CGUIVideoDialog() 
            : CRRCDialog(), combo_res(NULL), slider_autozoom(NULL), slider_texoff(NULL)
{
  // save current values to detect changes in the callback
  use_fs = cfgfile->getInt("video.fullscreen.fUse", 0);
  full_x = cfgfile->getInt("video.resolution.fullscreen.x", 800);
  full_y = cfgfile->getInt("video.resolution.fullscreen.y", 600);
  window_x = cfgfile->getInt("video.resolution.window.x", 800);
  window_y = cfgfile->getInt("video.resolution.window.y", 600);
  use_smart = cfgfile->getInt("video.camera.smart", 0);

  resolutions=NULL;
            
  combo_res       = new puaComboBox(LABEL_W + DLG_DEF_SPACE, 
                                   TOP_WIDGET_Y - DLG_DEF_SPACE - DLG_CHECK_H - COMBO_H,
                                   LABEL_W + COMBO_W,
                                   TOP_WIDGET_Y - DLG_DEF_SPACE - DLG_CHECK_H,
                                   NULL, true);
  combo_res->setChildColourScheme(PUCLASS_POPUPMENU, dlgCol1[0], dlgCol1[1], dlgCol1[2]);
  combo_res->setLabelPlace(PUPLACE_LOWER_LEFT);
  combo_res->setLabel(_("Resolution"));
  combo_res->setUserData(this);
  updateListeResolutions(use_fs);//=> setCurrentItem and newList(resolutions)
  combo_res->reveal();
  combo_res->activate();
  
  fs_check = new puButton(LABEL_W + DLG_DEF_SPACE,
                          TOP_WIDGET_Y - DLG_CHECK_H,
                          LABEL_W + DLG_DEF_SPACE + DLG_CHECK_W,
                          TOP_WIDGET_Y);
  fs_check->setLabelPlace(PUPLACE_CENTERED_LEFT);
  fs_check->setLabel(_("Fullscreen"));
  fs_check->setButtonType(PUBUTTON_VCHECK);
  fs_check->setUserData(this);
  fs_check->setCallback(CGUIVideoFSCallback);
  if (use_fs)
    fs_check->setValue(1);
  else
    fs_check->setValue(0);
  fs_check->reveal();
  
  slider_autozoom = new crrcSlider( LABEL_W + DLG_DEF_SPACE,
                                    TOP_WIDGET_Y - 2*DLG_DEF_SPACE - DLG_CHECK_H - COMBO_H - SLIDER_H,
                                    LABEL_W + DLG_DEF_SPACE + SLIDER_W,
                                    TOP_WIDGET_Y - 2*DLG_DEF_SPACE - DLG_CHECK_H - COMBO_H,
                                    NUM_W);
  slider_autozoom->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_autozoom->setLabel(_("Autozoom"));
  slider_autozoom->setMaxValue(AUTOZOOM_MAX);
  slider_autozoom->setStepSize(0.01);
  slider_autozoom->setValue(flAutozoom);
  
  // get a pointer to the video settings
  SimpleXMLTransfer *video = cfgfile->getChild("video", true);

  // frame rate limiter setting
  int iFPS = video->attributeAsInt("fps", DEFAULT_GAME_SPEED);
  slider_fps = new crrcSlider(LABEL_W + DLG_DEF_SPACE,
                              TOP_WIDGET_Y - 3*DLG_DEF_SPACE - DLG_CHECK_H - COMBO_H - 2*SLIDER_H,
                              LABEL_W + DLG_DEF_SPACE + SLIDER_W,
                              TOP_WIDGET_Y - 3*DLG_DEF_SPACE - DLG_CHECK_H - COMBO_H - SLIDER_H,
                              NUM_W);
  slider_fps->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_fps->setLabel(_("FPS limit"));
  slider_fps->setMinValue(FPS_MIN);
  slider_fps->setMaxValue(FPS_MAX);
  slider_fps->setStepSize(1);
  slider_fps->setValue(iFPS);

  // skybox texture offset setting
  dOldSkyTexOff = video->getDouble("skybox.texture_offset", DEFAULT_SKYBOX_TEXTURE_OFFSET);
  dOldSkyTexOff = dOldSkyTexOff * 10000.0;
  slider_texoff = new crrcSlider( LABEL_W + DLG_DEF_SPACE,
                                  TOP_WIDGET_Y - 4*DLG_DEF_SPACE - DLG_CHECK_H - COMBO_H - 3*SLIDER_H,
                                  LABEL_W + DLG_DEF_SPACE + SLIDER_W,
                                  TOP_WIDGET_Y - 4*DLG_DEF_SPACE - DLG_CHECK_H - COMBO_H - 2*SLIDER_H,
                                  NUM_W);
  slider_texoff->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_texoff->setLabel(_("Skybox fix"));
  slider_texoff->setMinValue(0);
  slider_texoff->setMaxValue(99);
  slider_texoff->setStepSize(1);
  slider_texoff->setValue((float)dOldSkyTexOff);

  // smart camera setting
  slider_smartcam = new crrcSlider( LABEL_W + DLG_DEF_SPACE,
                                    TOP_WIDGET_Y - 5*DLG_DEF_SPACE - DLG_CHECK_H - COMBO_H - 4*SLIDER_H,
                                    LABEL_W + DLG_DEF_SPACE + SLIDER_W,
                                    TOP_WIDGET_Y - 5*DLG_DEF_SPACE - DLG_CHECK_H - COMBO_H - 3*SLIDER_H,
                                    NUM_W);
  slider_smartcam->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_smartcam->setLabel(_("Smart camera"));
  slider_smartcam->setMinValue(0);
  slider_smartcam->setMaxValue(1);
  slider_smartcam->setStepSize(0.1);
  slider_smartcam->setValue(Video::getSmartCam());
  slider_smartcam->setUserData(this);
  slider_smartcam->setCallback(CGUIVideoSmartCallback);
  
  // sloppy camera setting
  slider_sloppycam = new crrcSlider( LABEL_W + DLG_DEF_SPACE,
                                     TOP_WIDGET_Y - 6*DLG_DEF_SPACE - DLG_CHECK_H - COMBO_H - 5*SLIDER_H,
                                     LABEL_W + DLG_DEF_SPACE + SLIDER_W,
                                     TOP_WIDGET_Y - 6*DLG_DEF_SPACE - DLG_CHECK_H - COMBO_H - 4*SLIDER_H,
                                     NUM_W);
  slider_sloppycam->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_sloppycam->setLabel(_("Sloppy camera"));
  slider_sloppycam->setMinValue(0);
  slider_sloppycam->setMaxValue(0.3);
  slider_sloppycam->setStepSize(0.05);
  slider_sloppycam->setValue(Video::getSloppyCam());
  slider_sloppycam->setUserData(this);
  slider_sloppycam->setCallback(CGUIVideoSloppyCallback);

  flSmartCam = slider_smartcam->getFloatValue();
  flSloppyCam = slider_sloppycam->getFloatValue();
  if (flSmartCam > 0)
    slider_sloppycam->setValue(0);
  if (flSloppyCam > 0)
    slider_smartcam->setValue(0);
  
  /**
   *  \todo widgets for color depth, shading, textures and mipmaps
   */  
  
  // finalize the dialog
  close();
  setSize(SLIDER_W + LABEL_W + 2*DLG_DEF_SPACE, 
          TOP_WIDGET_Y + 2*DLG_DEF_SPACE);
  setCallback(CGUIVideoCallback);
  centerOnScreen();
  reveal();
}


/**
 * Destroy the dialog.
 */

CGUIVideoDialog::~CGUIVideoDialog()
{
  freeListeResolutions();
}


/** \brief The dialog's callback.
 *
 */
void CGUIVideoCallback(puObject *obj)
{
  CGUIVideoDialog* dlg = (CGUIVideoDialog*)obj;

  if (obj->getIntegerValue() == CRRC_DIALOG_OK)
  {
    // Dialog left by clicking OK
    {
      std::string resolution = dlg->combo_res->getStringValue();
      if (resolution=="auto") resolution="0x0";
      if (resolution.find('x') != std::string::npos)
      {
        // String might be valid
        std::string resx = resolution.substr(0, resolution.find('x'));
        std::string resy = resolution.substr(resolution.find('x')+1);
        int iResX = atoi(resx.c_str());
        int iResY = atoi(resy.c_str());
        
        int new_use_fs = dlg->fs_check->getIntegerValue();
        
        // now compare the new values to the old values to detect changes
        bool values_changed = false;
        
        if (new_use_fs != dlg->use_fs)
        {
          values_changed = true;
        }
        else
        {
          if (dlg->use_fs)
          {
            if ((iResX != dlg->full_x) || (iResY != dlg->full_y))
            {
              values_changed = true;
            }
          }
          else
          {
            if ((iResX != dlg->window_x) || (iResY != dlg->window_y))
            {
              values_changed = true;
            }
          }
        }

        /* change detected? then store all values */
        if (values_changed)
        {
          std::cout << "Screen resolution changed to " << resx << "x" << resy;
          std::cout << ((new_use_fs) ? " (fullscreen)" : " (windowed)") << std::endl;
          if (new_use_fs)
          {
            cfgfile->setAttributeOverwrite("video.resolution.fullscreen.x", resx);
            cfgfile->setAttributeOverwrite("video.resolution.fullscreen.y", resy);
          }
          else
          {
            cfgfile->setAttributeOverwrite("video.resolution.window.x", resx);
            cfgfile->setAttributeOverwrite("video.resolution.window.y", resy);
          }

          cfgfile->setAttributeOverwrite("video.fullscreen.fUse", new_use_fs);
          
          #if defined linux
          // On Linux we can switch the mode on-the-fly
          Video::setupScreen(atoi(resx.c_str()), atoi(resy.c_str()), new_use_fs);
          #else
          // other platforms need a restart
          // Note: could work on Win as well, except if the user has resized the 
          // windows to full-size using the top-right corner window icon.
          new CGUIMsgBox(_("Please save your configuration and restart CRRCSim!"));
          #endif
        }
      }
    }
    flAutozoom = dlg->slider_autozoom->getFloatValue();
    
    Uint16 fps = dlg->slider_fps->getIntegerValue();
    crrc_time->setGameSpeed(fps);
    crrc_time->putBackIntoCfg(cfgfile);
    
    double dSkyTexOff = 0.0001 * dlg->slider_texoff->getFloatValue();
    if (dSkyTexOff != dlg->dOldSkyTexOff)
    {
      cfgfile->setAttributeOverwrite( "video.skybox.texture_offset",
                                      ftoStr(dSkyTexOff, 1, 4));
      Global::scenery->setSky(Video::setup_sky(NULL));
    }
    //
    Video::setSmartCam(dlg->slider_smartcam->getFloatValue());
    cfgfile->setAttributeOverwrite("video.camera.smart",
                                    doubleToString(Video::getSmartCam()));
    Video::setSloppyCam(dlg->slider_sloppycam->getFloatValue());
    cfgfile->setAttributeOverwrite("video.camera.sloppy",
                                    doubleToString(Video::getSloppyCam()));
  }
  
  puDeleteObject(obj);
}
/*********
*free previous malloc
*
**********/

void CGUIVideoDialog::freeListeResolutions()
{
  if(resolutions) for (int n=0; n<nResolutions; n++)
  {
    if(resolutions[n]) free(resolutions[n]);
  }      
  free (resolutions);
}

/*********
*create liste from standard+current resolutions and apply to ComboBox
*
*   parameter : fs =  1 for "full screen" resolutions, 
*                     0 for "windowed" resolutions
**********/
void CGUIVideoDialog::updateListeResolutions(int fs)
{
  
  //free previous list
  freeListeResolutions();
  
  // Generate list and set current value
  // Muss man den aktuellen Wert getrennt in die Liste tun?
  int nIdx = -1;
  {
    if (fs)
    {
      for (int n=0; n<stdResCnt; n++)
      {
        if (stdResX[n] == full_x && stdResY[n] == full_y)
          nIdx = n;
      }
    }
    else
    {
      for (int n=0; n<stdResCnt; n++)
      {
        if ((stdResX[n] == window_x) && (stdResY[n] == window_y))
          nIdx = n;
      }
    }
  if (fs && (full_x==0)&&(full_y==0) )//auto resolution
      nIdx= stdResCnt;
  }
  nResolutions = stdResCnt;
  if (nIdx == -1)
  {
    nResolutions++;         //add item for no standard current resolution
    nIdx=nResolutions - 1;  //
  }
  if (fs) nResolutions++; //for item "auto"

  resolutions = (char**)malloc(sizeof(char**) * (nResolutions+1));

  // set list of resolutions 
  {
    std::string name;
    
    for (int n=0; n<nResolutions; n++)
    {
      if(fs && (n == (nResolutions-1))) name = "auto";
      else if (n == stdResCnt)
      { //no standard current resolution
        if (fs)
          {
          name = cfgfile->getString("video.resolution.fullscreen.x") + "x" +
                 cfgfile->getString("video.resolution.fullscreen.y");
          if(name=="0x0")name = "auto";
          }
        else
          name = cfgfile->getString("video.resolution.window.x") + "x" +
                 cfgfile->getString("video.resolution.window.y");
      }
      else
      { //standard resolution
        name = itoStr(stdResX[n], ' ', 1) + "x" + itoStr(stdResY[n], ' ', 1);
      }
      resolutions[n] = (char*)malloc(sizeof(char**) * (name.length()+1));
      strcpy(resolutions[n], name.c_str());
    }      
    resolutions[nResolutions] = (char*)0;
  }
  //apply to comboBox
  combo_res->newList(resolutions) ;
  combo_res->setCurrentItem(nIdx);
}

/***
* Callback for "full screen"/"windowed"  check button 
*
**/
static void CGUIVideoFSCallback(puObject *obj)
{
  CGUIVideoDialog* dlg   = (CGUIVideoDialog*)(obj->getUserData());
  int fs = obj->getIntegerValue();
  dlg->updateListeResolutions(fs);
}

/** 
 *  Callback for the "smart camera" slider.
 */
void CGUIVideoSmartCallback(puObject *obj)
{
  CGUIVideoDialog* dlg   = (CGUIVideoDialog*)(obj->getUserData());
 
  // to catch odd behaviour of slider around 0
  if (obj->getFloatValue() < 0.001)
    dlg->slider_smartcam->setValue(0);

  dlg->flSmartCam = obj->getFloatValue();
  
  if (obj->getFloatValue() > 0)
    dlg->slider_sloppycam->setValue(0);
  else
    dlg->slider_sloppycam->setValue(dlg->flSloppyCam);
}

/** 
 *  Callback for the "sloppy camera" slider.
 */
void CGUIVideoSloppyCallback(puObject *obj)
{
  CGUIVideoDialog* dlg   = (CGUIVideoDialog*)(obj->getUserData());
  
  // to catch odd behaviour of slider around 0
  if (obj->getFloatValue() < 0.001)
    dlg->slider_sloppycam->setValue(0);

  dlg->flSloppyCam = obj->getFloatValue();
  
  if (obj->getFloatValue() > 0)
    dlg->slider_smartcam->setValue(0);
  else
    dlg->slider_smartcam->setValue(dlg->flSmartCam);
}
