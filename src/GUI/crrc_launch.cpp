/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2008, 2009 Jens Wilhelm Wulf (original author)
 * Copyright (C) 2006, 2008 Jan Reucker
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
 *  \file crrc_launch.cpp
 *  Implementation of class CGUILaunchDialog
 */

#include "../i18n.h"
#include "crrc_launch.h"

#include <stdio.h>
#include <stdlib.h>

#include "util.h"
#include "../global.h"
#include "../aircraft.h"
#include "../mod_fdm/fdm.h"
#include "../config.h"
#include "../crrc_main.h"
#include "../mod_landscape/crrc_scenery.h"
#include "../mod_misc/lib_conversions.h"
#include "../mod_misc/SimpleXMLTransfer.h"

static void CGUILaunchCallback(puObject *obj);
static void CGUILaunchPresetCallback(puObject *obj);
static void CGUILaunchNewPresetCallback(puObject *obj);
static void CGUILaunchVelSliderCallback(puObject *obj);
static void CGUILaunchReltoPlayer(puObject *obj);
static void CGUILaunchNoreltoPlayer(puObject *obj);

#define BUTTON_BOX_HEIGHT   (2*DLG_DEF_SPACE+DLG_DEF_BUTTON_HEIGHT)
#define SHORT_SLIDER_W      (210)
#define SLIDER_W            330
#define SLIDER_H            DLG_DEF_BUTTON_HEIGHT
#define COMBO_W             330
#define COMBO_H             DLG_DEF_BUTTON_HEIGHT
#define LABEL_W             170
#define NUM_W               70


CGUILaunchDialog::CGUILaunchDialog() 
            : CRRCDialog()
{
int current_start_index=0;
int RelToPlayer;
  // Load presets
    // first copy those from the config file
  presetGrp = new SimpleXMLTransfer(cfgfile->getChild("launch"));

    // now add those from the current airplane
  SimpleXMLTransfer *alp = Global::aircraft->getFDMInterface()->getLaunchPresets();
  if (alp != NULL)
  {
    for (int i = 0; i < alp->getChildCount(); i++)
    {
      // don't add existing children, always make a copy!
      presetGrp->addChild(new SimpleXMLTransfer(alp->getChildAt(i)));
    }
  }
    // make a list for the preset combo box
  presets = T_GUI_Util::loadnames(presetGrp, nPresets);
  
    //load scenery start positions
  std::string current_start_name = cfg->getCurLocCfgPtr(cfgfile)->getString("start.position","");
  NumStartPosition = Global::scenery->getNumStartPosition();
  if (NumStartPosition != 0)
    {
    names_start_position = new char*[NumStartPosition+1];
    for (int i=0; i < NumStartPosition; i++)
      {
      std::string *const name = Global::scenery->getStartPositionName(i);
      if (*name == current_start_name) current_start_index = i;
      char *cstr = new char [name->size()+1];
      strcpy (cstr, name->c_str());
      names_start_position[i] = cstr;
      }
    names_start_position[NumStartPosition] = NULL;
    }
  else names_start_position = NULL;
  if (NumStartPosition != 0)
        RelToPlayer = cfgfile->getInt("launch.rel_to_player");
  else  RelToPlayer = 1;

  // Create Widgets (construct in the inverse order of their display)
  int y_in_win = BUTTON_BOX_HEIGHT;
      // save preset
  inputNewName = new puInput(          DLG_DEF_SPACE, y_in_win,
                             LABEL_W + DLG_DEF_SPACE, y_in_win + SLIDER_H);
  inputNewName->setValue(_("Name of new preset"));
  puOneShot* buttonTmp = new puOneShot(LABEL_W + DLG_DEF_SPACE,             y_in_win,
                                       LABEL_W + DLG_DEF_SPACE + SLIDER_W,  y_in_win + SLIDER_H);
  y_in_win += ( SLIDER_H + DLG_DEF_SPACE );                              
  buttonTmp->setLegend(_("Save as new preset"));
  buttonTmp->setCallback(CGUILaunchNewPresetCallback);
  buttonTmp->setUserData(this); 
 
      // choice of start position 
  comboStartPos = new puaComboBox(LABEL_W + DLG_DEF_SPACE,   y_in_win,
                                 LABEL_W + COMBO_W,         y_in_win + SLIDER_H,
                                 NULL, false); 
  y_in_win += ( SLIDER_H + DLG_DEF_SPACE ); 
  comboStartPos->setChildColourScheme(PUCLASS_POPUPMENU, dlgCol1[0], dlgCol1[1], dlgCol1[2]);
  comboStartPos->newList(names_start_position);
  comboStartPos->setLabelPlace(PUPLACE_CENTERED_LEFT);
  //comboStartPos->setLabel("Start position");if more 2 positions this label is badly positioned (bug PLIB ?)
  comboStartPos->setCurrentItem(current_start_index);
  comboStartPos->setUserData(this);
  
        // check launch from start
  check_norel_to_player = new puButton(LABEL_W + DLG_DEF_SPACE              ,   y_in_win,
                                       LABEL_W + DLG_DEF_SPACE + DLG_CHECK_W,   y_in_win + DLG_CHECK_H);
  y_in_win += ( DLG_CHECK_H + DLG_DEF_SPACE ); 
  check_norel_to_player->setButtonType(PUBUTTON_VCHECK);
  check_norel_to_player->setLabelPlace(PUPLACE_CENTERED_LEFT);
  check_norel_to_player->setLabel(_("Launch from start"));
  check_norel_to_player->setValue(! RelToPlayer);
  check_norel_to_player->setUserData(this);;
  check_norel_to_player->setCallback(CGUILaunchNoreltoPlayer);

      // slider relative right position
  slider_rel_right = new crrcSlider(LABEL_W + DLG_DEF_SPACE,                 y_in_win,
                                    LABEL_W + DLG_DEF_SPACE + SLIDER_W, y_in_win + SLIDER_H,
                                    NUM_W);
  y_in_win += ( SLIDER_H + DLG_DEF_SPACE );                                 
  slider_rel_right->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_rel_right->setLabel(_("Relative right position"));
  slider_rel_right->setSliderFraction(0.05);
  slider_rel_right->setMinValue(-50);
  slider_rel_right->setMaxValue( 50);
  slider_rel_right->setStepSize(1);
  slider_rel_right->setValue((float)cfgfile->getDouble("launch.rel_right"));
  
       // slider relative front position  widgets"comboStartPos" and "slider_rel_front" superposed   
  slider_rel_front = new crrcSlider(LABEL_W + DLG_DEF_SPACE,                  y_in_win,
                                    LABEL_W + DLG_DEF_SPACE + SLIDER_W, y_in_win + SLIDER_H,
                                    NUM_W);
  y_in_win += ( SLIDER_H + DLG_DEF_SPACE );                                                    
  slider_rel_front->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_rel_front->setLabel(_("Relative front position"));
  slider_rel_front->setSliderFraction(0.05);
  slider_rel_front->setMinValue(  0);
  slider_rel_front->setMaxValue(100);
  slider_rel_front->setStepSize(1);
  slider_rel_front->setValue((float)cfgfile->getDouble("launch.rel_front"));
 
        // check relative to player
  check_rel_to_player =   new puButton(LABEL_W + DLG_DEF_SPACE,                y_in_win,
                                       LABEL_W + DLG_DEF_SPACE + DLG_CHECK_W,  y_in_win + DLG_CHECK_H);
  y_in_win += ( DLG_CHECK_H + DLG_DEF_SPACE ); 
  check_rel_to_player->setButtonType(PUBUTTON_VCHECK);
  check_rel_to_player->setLabelPlace(PUPLACE_CENTERED_LEFT);
  check_rel_to_player->setLabel(_("Launch from player"));
  check_rel_to_player->setValue( RelToPlayer );
  check_rel_to_player->setUserData(this);
  check_rel_to_player->setCallback(CGUILaunchReltoPlayer);
  updateSliderVisibility();

      // slider vertical angle
  slider_angle = new crrcSlider(LABEL_W + DLG_DEF_SPACE,            y_in_win,
                                LABEL_W + DLG_DEF_SPACE + SLIDER_W, y_in_win + SLIDER_H,
                                NUM_W);
  y_in_win += ( SLIDER_H + DLG_DEF_SPACE );
  slider_angle->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_angle->setLabel(_("Vertical angle [rad]"));
  slider_angle->setSliderFraction(0.05);
  slider_angle->setMinValue(-0.78);
  slider_angle->setMaxValue(1.57);
  slider_angle->setStepSize(0.01);
  slider_angle->setValue((float)cfgfile->getDouble("launch.angle"));
  
      // check sal
  check_dlg = new puButton(LABEL_W + DLG_DEF_SPACE,               y_in_win,
                           LABEL_W + DLG_DEF_SPACE + DLG_CHECK_W, y_in_win + DLG_CHECK_H );
  y_in_win += ( DLG_CHECK_H + DLG_DEF_SPACE ); 
  check_dlg->setButtonType(PUBUTTON_VCHECK);
  check_dlg->setLabelPlace(PUPLACE_CENTERED_LEFT);
  check_dlg->setLabel(_("Simulate SAL"));
  check_dlg->setValue((int)cfgfile->getInt("launch.sal"));

      // slider velocity
  slider_velocity = new crrcSlider(LABEL_W + DLG_DEF_SPACE,                   y_in_win,
                                   LABEL_W + DLG_DEF_SPACE + SHORT_SLIDER_W,  y_in_win + SLIDER_H,
                                   NUM_W);
  slider_velocity->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_velocity->setLabel(_("Relative velocity"));
  slider_velocity->setSliderFraction(0.05);
  slider_velocity->setMinValue(0);
  slider_velocity->setMaxValue(7);
  slider_velocity->setStepSize(0.1);
  slider_velocity->setUserData(this);
  slider_velocity->setValue((float)cfgfile->getDouble("launch.velocity_rel"));
  slider_velocity->setCallback(CGUILaunchVelSliderCallback);
  text_velocity_abs = new puText( LABEL_W + 2*DLG_DEF_SPACE + SHORT_SLIDER_W,  y_in_win);
  y_in_win += ( SLIDER_H + DLG_DEF_SPACE );
  updateVelAbs();
  
      // slider altitude
  slider_altitude = new crrcSlider(LABEL_W + DLG_DEF_SPACE,            y_in_win,
                                   LABEL_W + DLG_DEF_SPACE + SLIDER_W, y_in_win + SLIDER_H,
                                   NUM_W);
  y_in_win += ( SLIDER_H + DLG_DEF_SPACE );
  slider_altitude->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_altitude->setLabel(_("Altitude [ft]"));
  slider_altitude->setSliderFraction(0.05);
  slider_altitude->setMinValue(0);
  slider_altitude->setMaxValue(700);
  slider_altitude->setStepSize(1);
  slider_altitude->setValue((float)cfgfile->getDouble("launch.altitude"));

      //Presetchoise
  comboPresets = new puaComboBox(LABEL_W + DLG_DEF_SPACE,   y_in_win,
                                 LABEL_W + COMBO_W,         y_in_win + SLIDER_H,
                                 NULL, false);
  y_in_win += ( SLIDER_H + DLG_DEF_SPACE );                               
  comboPresets->setChildColourScheme(PUCLASS_POPUPMENU, dlgCol1[0], dlgCol1[1], dlgCol1[2]);
  comboPresets->newList(presets);
  comboPresets->setLabelPlace(PUPLACE_CENTERED_LEFT);
  comboPresets->setLabel(_("Load Preset"));
  comboPresets->setCurrentItem(0);
  comboPresets->setCallback(CGUILaunchPresetCallback);
  comboPresets->reveal();
  comboPresets->activate();
  comboPresets->setUserData(this); 
 
  // finalize the dialog
  close();
  setSize(SLIDER_W + LABEL_W + 2*DLG_DEF_SPACE, y_in_win);
  setCallback(CGUILaunchCallback);
  centerOnScreen();
  reveal();
}


/**
 * Destroy the dialog.
 */

CGUILaunchDialog::~CGUILaunchDialog()
{
  for (int n=0; n<nPresets; n++)
    free(presets[n]);
  free(presets);
  
  if(names_start_position) 
     {
     for (int i=0; i<NumStartPosition; i++)
            delete names_start_position[i];
     delete names_start_position;
     }
}


/** \brief Update absolute velocity label
 *
 */
void CGUILaunchDialog::updateVelAbs()
{
  // get trimmed flight velocity
  double dTrimmedFlightVelocity = Global::aircraft->getFDM()->getTrimmedFlightVelocity();
  float fValue = (float)(slider_velocity->getFloatValue() * dTrimmedFlightVelocity);
  snprintf(acVelAbs, PUSTRING_MAX, "= %.1f ft/s", fValue);
  text_velocity_abs->setLabel(acVelAbs);
}


/** \brief Callback when moving the velocity slider
 *
 *
 */
void CGUILaunchVelSliderCallback(puObject *obj)
{
  CGUILaunchDialog* dlg   = (CGUILaunchDialog*)(obj->getUserData());
  dlg->updateVelAbs();
}
/*\brief callback when toogle rel_to_player
 *
 */
void CGUILaunchReltoPlayer(puObject *obj)
{
  int check;
  CGUILaunchDialog* dlg   = (CGUILaunchDialog*)(obj->getUserData());
  if(dlg->NumStartPosition == 0)
        check=1;//if no start position en scenery, force check_rel_to_player to 1
  else  check = obj->getIntegerValue();
  dlg->check_rel_to_player->setValue(check);
  dlg->check_norel_to_player->setValue(!check);
  dlg->updateSliderVisibility();
}
void CGUILaunchNoreltoPlayer(puObject *obj)
{
  int check;
  CGUILaunchDialog* dlg   = (CGUILaunchDialog*)(obj->getUserData());
  if(dlg->NumStartPosition == 0)
        check=0;//if no start position en scenery, force check_rel_to_player to 1
  else  check = obj->getIntegerValue();
  dlg->check_rel_to_player->setValue(!check);
  dlg->check_norel_to_player->setValue(check);
  dlg->updateSliderVisibility();
}

void CGUILaunchDialog::updateSliderVisibility()
{
  if(check_rel_to_player->getIntegerValue())
    {
    comboStartPos->hide();
    slider_rel_front->reveal();
    slider_rel_right->reveal();
    }
  else
    {
    comboStartPos->reveal();
    slider_rel_front->hide();
    slider_rel_right->hide();
    }
}

/** \brief callback to load a preset.
 *
 */
void CGUILaunchPresetCallback(puObject *obj)
{
  CGUILaunchDialog* dlg   = (CGUILaunchDialog*)obj->getUserData();
  int               nItem = dlg->comboPresets->getCurrentItem();
    
  if (nItem == 0)
  {
    // the "default" entry
    dlg->slider_altitude->setValue(0);
    dlg->slider_velocity->setValue(0);
    dlg->slider_angle->setValue(0);
    dlg->check_dlg->setValue(0);
    dlg->check_rel_to_player->setValue(1);
    dlg->check_norel_to_player->setValue(0);
    dlg->slider_rel_front->setValue(MODELSTART_REL_FRONT);
    dlg->slider_rel_right->setValue(MODELSTART_REL_RIGHT);
  }
  else
  {
    SimpleXMLTransfer* preset = dlg->presetGrp->getChildAt(nItem-1);
    dlg->slider_altitude->setValue((float)preset->getDouble("altitude", 10.0));
    dlg->slider_velocity->setValue((float)preset->getDouble("velocity_rel", 1.0));
    dlg->slider_angle->setValue((float)preset->getDouble("angle", 0.0));
    dlg->check_dlg->setValue((int)preset->getInt("sal", 0));
    int rel_to_player = preset->getInt("rel_to_player", 1);
    dlg->check_rel_to_player->setValue( rel_to_player);
    dlg->check_norel_to_player->setValue(!rel_to_player);
    dlg->slider_rel_front->setValue((float)preset->getDouble("rel_front", MODELSTART_REL_FRONT));
    dlg->slider_rel_right->setValue((float)preset->getDouble("rel_right", MODELSTART_REL_RIGHT));
  }
  dlg->updateVelAbs();
  dlg->updateSliderVisibility();
}


/** \brief callback to save a preset.
 *
 */
void CGUILaunchNewPresetCallback(puObject *obj)
{
  CGUILaunchDialog* dlg   = (CGUILaunchDialog*)obj->getUserData();
  
  SimpleXMLTransfer* launch   = new SimpleXMLTransfer();
  
  launch->setName("preset");
  launch->setAttribute("name_en",       dlg->inputNewName->getStringValue());
  launch->setAttribute("altitude",      dlg->slider_altitude->getStringValue());
  launch->setAttribute("velocity_rel",  dlg->slider_velocity->getStringValue());
  launch->setAttribute("angle",         dlg->slider_angle->getStringValue());
  launch->setAttribute("sal",           dlg->check_dlg->getStringValue());
  launch->setAttribute("rel_to_player", dlg->check_rel_to_player->getStringValue());
  launch->setAttribute("rel_front",     dlg->slider_rel_front->getStringValue());
  launch->setAttribute("rel_right",     dlg->slider_rel_right->getStringValue());
  
  cfgfile->getChild("launch", true)->addChild(launch);
  
  // save file
  options_saveToFile();
  
  // exit dialog (otherwise we would have to regenerate presets)
  dlg->setValue(CRRC_DIALOG_OK);
  CGUILaunchCallback(dlg);
}

/** \brief The dialog's callback.
 *
 */
void CGUILaunchCallback(puObject *obj)
{
  CGUILaunchDialog *dlg = (CGUILaunchDialog*)obj;

  if (obj->getIntegerValue() == CRRC_DIALOG_OK)
  {
    // Dialog left by clicking OK
    cfgfile->setAttributeOverwrite("launch.altitude", 
                                   dlg->slider_altitude->getStringValue());

    cfgfile->setAttributeOverwrite("launch.velocity_rel", 
                                   dlg->slider_velocity->getStringValue());
    
    cfgfile->setAttributeOverwrite("launch.angle", 
                                   dlg->slider_angle->getStringValue());
    
    cfgfile->setAttributeOverwrite("launch.sal", dlg->check_dlg->getStringValue());

    cfgfile->setAttributeOverwrite("launch.rel_to_player", dlg->check_rel_to_player->getStringValue());
    cfgfile->setAttributeOverwrite("launch.rel_front",     dlg->slider_rel_front->getStringValue());
    cfgfile->setAttributeOverwrite("launch.rel_right",     dlg->slider_rel_right->getStringValue());
    
    cfg->getCurLocCfgPtr(cfgfile)->setAttributeOverwrite("start.position",
                                                   dlg->comboStartPos->getStringValue());
  }
  delete dlg->presetGrp;
  puDeleteObject(obj);
}
