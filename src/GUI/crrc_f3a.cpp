/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2008 Olivier Bordes (original author)
 * Copyright (C) 2005, 2006, 2008 Jan Reucker
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
  

// F3A dialog
#include "../i18n.h"
#include "crrc_f3a.h"

#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>

#include "../global.h"
#include "util.h"
#include "../crrc_main.h"
#include "../mod_misc/lib_conversions.h"
#include "../mod_misc/SimpleXMLTransfer.h"
#include "../mod_mode/T_GameHandler.h"
#include "../mod_mode/F3A/handlerF3A.h"

static void CGUIF3ADialogCallback(puObject * obj);


#define BUTTON_BOX_HEIGHT   (2*DLG_DEF_SPACE+DLG_DEF_BUTTON_HEIGHT)
#define SLIDER_W            308
#define SLIDER_H            DLG_DEF_BUTTON_HEIGHT
#define COMBO_H             DLG_DEF_BUTTON_HEIGHT
#define COMBO_W             330
#define LABEL_W             250
#define NUM_W               70


CGUIF3ADialog::CGUIF3ADialog():CRRCDialog()
{
  int f3a_cfg_enabled;
  int y_in_win = 0;

  // assure that all attributes exist in the config file
  HandlerF3A::prepareConfigFile(cfgfile);
  SimpleXMLTransfer* f3acfg = cfg->getCurLocCfgPtr(cfgfile)->getChild("game.f3a", true);
  
  f3a_cfg_enabled = cfgfile->getInt("game.f3a.enabled");
  
  // construct the button in the inverse order of their display
  y_in_win = BUTTON_BOX_HEIGHT;

  // ------------ save a new preset
  //inputNewName = new puInput(DLG_DEF_SPACE,y_in_win, 
  //LABEL_W + DLG_DEF_SPACE, y_in_win + COMBO_H);
  //inputNewName->setValue("name of new preset");

  //puOneShot* buttonTmp = new puOneShot(LABEL_W + DLG_DEF_SPACE,y_in_win, 
  //LABEL_W + DLG_DEF_SPACE + SLIDER_W, y_in_win + COMBO_H);
  //y_in_win+=COMBO_H + DLG_DEF_SPACE  ;
  //buttonTmp->setLegend("Save as new preset");
  //buttonTmp->setCallback(CGUIF3ADialogNewPresetCallback);
  //buttonTmp->setUserData(this);      

  // ------------ draw indicators for pitch & roll
  f3a_draw_indicators = new puButton(LABEL_W + DLG_DEF_SPACE, y_in_win,
                                     LABEL_W + DLG_DEF_SPACE + DLG_CHECK_W,
                                     y_in_win + DLG_CHECK_H);
  y_in_win += DLG_CHECK_H + DLG_DEF_SPACE;
  f3a_draw_indicators->setLabelPlace(PUPLACE_CENTERED_LEFT);
  f3a_draw_indicators->setLabel(_("Show pitch & roll indicators"));
  f3a_draw_indicators->setButtonType(PUBUTTON_VCHECK);
  f3a_draw_indicators->setValue(cfgfile->getInt("game.f3a.draw_indicators"));

  // ------------ draw airplane trajectory
  f3a_draw_trajectory = new puButton(LABEL_W + DLG_DEF_SPACE, y_in_win,
                                     LABEL_W + DLG_DEF_SPACE + DLG_CHECK_W,
                                     y_in_win + DLG_CHECK_H);
  y_in_win += DLG_CHECK_H + DLG_DEF_SPACE;
  f3a_draw_trajectory->setLabelPlace(PUPLACE_CENTERED_LEFT);
  f3a_draw_trajectory->setLabel(_("Show trajectory"));
  f3a_draw_trajectory->setButtonType(PUBUTTON_VCHECK);
  f3a_draw_trajectory->setValue(cfgfile->getInt("game.f3a.draw_trajectory"));

  // ------------ draw grid on flight plane
  f3a_draw_grid = new puButton(LABEL_W + DLG_DEF_SPACE, y_in_win,
                               LABEL_W + DLG_DEF_SPACE + DLG_CHECK_W,
                               y_in_win + DLG_CHECK_H);
  y_in_win += DLG_CHECK_H + DLG_DEF_SPACE;
  f3a_draw_grid->setLabelPlace(PUPLACE_CENTERED_LEFT);
  f3a_draw_grid->setLabel(_("Show reference grid"));
  f3a_draw_grid->setButtonType(PUBUTTON_VCHECK);
  f3a_draw_grid->setValue(cfgfile->getInt("game.f3a.draw_grid"));

  // ------------ attitude angle tolerance
  slider_tolerance = new crrcSlider(LABEL_W + DLG_DEF_SPACE, y_in_win,
                                    LABEL_W + DLG_DEF_SPACE + SLIDER_W,
                                    y_in_win + SLIDER_H, NUM_W);
  y_in_win += SLIDER_H + DLG_DEF_SPACE;
  slider_tolerance->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_tolerance->setLabel(_("Pitch & roll angle tolerance [deg]"));
  slider_tolerance->setSliderFraction(0.05);
  slider_tolerance->setMinValue(1);
  slider_tolerance->setMaxValue(10);
  slider_tolerance->setStepSize(1);
  slider_tolerance->setValue(cfgfile->getInt("game.f3a.attitude_tolerance"));

  // ------------ trajectory persistence
  slider_persistence = new crrcSlider(LABEL_W + DLG_DEF_SPACE, y_in_win,
                                      LABEL_W + DLG_DEF_SPACE + SLIDER_W,
                                      y_in_win + SLIDER_H, NUM_W);
  y_in_win += SLIDER_H + DLG_DEF_SPACE;
  slider_persistence->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_persistence->setLabel(_("Trajectory persistence [s]"));
  slider_persistence->setSliderFraction(0.05);
  slider_persistence->setMinValue(1);
  slider_persistence->setMaxValue(30);
  slider_persistence->setStepSize(1);
  slider_persistence->setValue(cfgfile->getInt("game.f3a.trajectory_persistence"));

  // location specifics parameters
  
  // ------------ grid size
  slider_grid_size = new crrcSlider(LABEL_W + DLG_DEF_SPACE, y_in_win,
                                    LABEL_W + DLG_DEF_SPACE + SLIDER_W,
                                    y_in_win + SLIDER_H, NUM_W);
  y_in_win += SLIDER_H + DLG_DEF_SPACE;
  slider_grid_size->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_grid_size->setLabel(_("Grid size [ft]"));
  slider_grid_size->setSliderFraction(0.05);
  slider_grid_size->setMinValue(50);   //  50ft = 15m
  slider_grid_size->setMaxValue(300);  // 300ft = 100m
  slider_grid_size->setStepSize(10);
  slider_grid_size->setValue(f3acfg->getInt("grid_size"));

  // ------------ flight height
  slider_flight_height = new crrcSlider(LABEL_W + DLG_DEF_SPACE, y_in_win,
                                        LABEL_W + DLG_DEF_SPACE + SLIDER_W,
                                        y_in_win + SLIDER_H, NUM_W);
  y_in_win += SLIDER_H + DLG_DEF_SPACE;
  slider_flight_height->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_flight_height->setLabel(_("Flight height [ft]"));
  slider_flight_height->setSliderFraction(0.05);
  slider_flight_height->setMinValue(0);
  slider_flight_height->setMaxValue(200);  // 200ft = 60m
  slider_flight_height->setStepSize(10);
  slider_flight_height->setValue(f3acfg->getInt("flight_height"));

  // ------------ flight distance
  slider_flight_distance = new crrcSlider(LABEL_W + DLG_DEF_SPACE, y_in_win,
                                          LABEL_W + DLG_DEF_SPACE + SLIDER_W,
                                          y_in_win + SLIDER_H, NUM_W);
  y_in_win += SLIDER_H + DLG_DEF_SPACE;
  slider_flight_distance->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_flight_distance->setLabel(_("Flight plane distance [ft]"));
  slider_flight_distance->setSliderFraction(0.05);
  slider_flight_distance->setMinValue(100);  // 100ft = 30 m
  slider_flight_distance->setMaxValue(500);  // 500ft = 150 m
  slider_flight_distance->setStepSize(50);
  slider_flight_distance->setValue(f3acfg->getInt("flight_distance"));
  
  // ------------ orientation
  slider_orientation = new crrcSlider(LABEL_W + DLG_DEF_SPACE, y_in_win,
                                         LABEL_W + DLG_DEF_SPACE + SLIDER_W,
                                         y_in_win + SLIDER_H, NUM_W);
  y_in_win += SLIDER_H + DLG_DEF_SPACE;
  slider_orientation->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_orientation->setLabel(_("Orientation [degrees]"));
  slider_orientation->setSliderFraction(0.05);
  slider_orientation->setMinValue(0);  
  slider_orientation->setMaxValue(360);
  slider_orientation->setStepSize(1);
  slider_orientation->setValue(f3acfg->getInt("orientation"));
  
   // ------------ position nord
  slider_position_n = new crrcSlider(LABEL_W + DLG_DEF_SPACE, y_in_win,
                                         LABEL_W + DLG_DEF_SPACE + SLIDER_W,
                                         y_in_win + SLIDER_H, NUM_W);
  y_in_win += SLIDER_H + DLG_DEF_SPACE;
  slider_position_n->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_position_n->setLabel(_("Center position North [ft]"));
  slider_position_n->setSliderFraction(0.05);
  slider_position_n->setMinValue(-600);  
  slider_position_n->setMaxValue(600);
  slider_position_n->setStepSize(10);
  slider_position_n->setValue(f3acfg->getInt("position_north"));

   // ------------ position east
  slider_position_e = new crrcSlider(LABEL_W + DLG_DEF_SPACE, y_in_win,
                                         LABEL_W + DLG_DEF_SPACE + SLIDER_W,
                                         y_in_win + SLIDER_H, NUM_W);
  y_in_win += SLIDER_H + DLG_DEF_SPACE;
  slider_position_e->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_position_e->setLabel(_("Center position East [ft]"));
  slider_position_e->setSliderFraction(0.05);
  slider_position_e->setMinValue(-600);  
  slider_position_e->setMaxValue(600);
  slider_position_e->setStepSize(10);
  slider_position_e->setValue(f3acfg->getInt("position_east"));

  // ------------ presets
  //  presetGrp = f3acfg->getChild("preset");    
  //  presets = T_GUI_Util::loadnames(presetGrp, nPresets);
  //
  //  comboPresets = new puaComboBox(LABEL_W + DLG_DEF_SPACE, y_in_win,
  //      LABEL_W + COMBO_W, y_in_win + COMBO_H, NULL, false);
  //  y_in_win+=COMBO_H + DLG_DEF_SPACE  ;
  //  comboPresets->setChildColourScheme(PUCLASS_POPUPMENU, dlgCol1[0], dlgCol1[1], dlgCol1[2]);
  //  comboPresets->newList(presets);
  //  comboPresets->setLabelPlace(PUPLACE_CENTERED_LEFT);
  //  comboPresets->setLabel("Load Preset");
  //  comboPresets->setCurrentItem(0);
  //  comboPresets->setCallback(CGUIF3ADialogPresetCallback);
  //  comboPresets->reveal();
  //  comboPresets->activate();
  //  comboPresets->setUserData(this);

  // ------------ enable F3A
  f3a_enable = new puButton(LABEL_W + DLG_DEF_SPACE, y_in_win,
                            LABEL_W + DLG_DEF_SPACE + DLG_CHECK_W,
                            y_in_win + DLG_CHECK_H);
  y_in_win += DLG_CHECK_H + DLG_DEF_SPACE;
  f3a_enable->setLabelPlace(PUPLACE_CENTERED_LEFT);
  f3a_enable->setLabel(_("Enable F3A mode"));
  f3a_enable->setButtonType(PUBUTTON_VCHECK);

  if (f3a_cfg_enabled == 1)
    f3a_enable->setValue(1);
  else
    f3a_enable->setValue(0);
  f3a_enable->reveal();

  //
  // sounds files
  // shadow mode
  // pilote position
  // zoom control
  // field of view

  // finalize the dialog
  close();
  setSize(COMBO_W + LABEL_W + 2 * DLG_DEF_SPACE, y_in_win);
  setCallback(CGUIF3ADialogCallback);
  centerOnScreen();
  reveal();
}


/**
 * Destroy the dialog.
 */

CGUIF3ADialog::~CGUIF3ADialog()
{
}


/** \brief The dialog's callback.
 *
 */
void CGUIF3ADialogCallback(puObject * obj)
{
  CGUIF3ADialog *dlg = (CGUIF3ADialog *) obj;

  if (obj->getIntegerValue() == CRRC_DIALOG_OK)
  {
    // create a new game handler depending on the "enabled" checkbox
    int f3a_is_enabled = dlg->f3a_enable->getIntegerValue();
    cfgfile->setAttributeOverwrite("game.f3a.enabled", f3a_is_enabled);
    if (f3a_is_enabled)
    {
      // if another game mode is currently active disable it
      if (Global::gameHandler->gameType() != std::string("F3A"))
        Global::gameHandler->disable();

      delete(Global::gameHandler);
      Global::gameHandler = new HandlerF3A();
    }
    else
    {
      // if this game mode is currently active disable it
      if (Global::gameHandler->gameType() == std::string("F3A"))
      {
        delete(Global::gameHandler);
        Global::gameHandler = new T_GameHandler();
      }
    }
    
    // in any case, save the values in the config file
    int indicators = dlg->f3a_draw_indicators->getIntegerValue();
    cfgfile->setAttributeOverwrite("game.f3a.draw_indicators", indicators);
    int trajectory = dlg->f3a_draw_trajectory->getIntegerValue();
    cfgfile->setAttributeOverwrite("game.f3a.draw_trajectory", trajectory);
    int grid = dlg->f3a_draw_grid->getIntegerValue();
    cfgfile->setAttributeOverwrite("game.f3a.draw_grid", grid);
    int tolerance = dlg->slider_tolerance->getIntegerValue();
    cfgfile->setAttributeOverwrite("game.f3a.attitude_tolerance", tolerance);
    int persistence = dlg->slider_persistence->getIntegerValue();
    cfgfile->setAttributeOverwrite("game.f3a.trajectory_persistence", persistence);
    
    // save location specifics parameters
    SimpleXMLTransfer* f3acfg = cfg->getCurLocCfgPtr(cfgfile)->getChild("game.f3a", true);
    int size = dlg->slider_grid_size->getIntegerValue();
    f3acfg->setAttributeOverwrite("grid_size", size);
    int height = dlg->slider_flight_height->getIntegerValue();
    f3acfg->setAttributeOverwrite("flight_height", height);
    int distance = dlg->slider_flight_distance->getIntegerValue();
    f3acfg->setAttributeOverwrite("flight_distance", distance);
    int orientation = dlg->slider_orientation->getIntegerValue();
    f3acfg->setAttributeOverwrite("orientation", orientation);
    int position_n = dlg->slider_position_n->getIntegerValue();
    f3acfg->setAttributeOverwrite("position_north", position_n);
    int position_e = dlg->slider_position_e->getIntegerValue();
    f3acfg->setAttributeOverwrite("position_east", position_e);
    
    // if F3A is enabled, also update the running game handler
    // with the new configuration values
    if (Global::gameHandler->gameType() == std::string("F3A"))
    {
      HandlerF3A *aHandler = (HandlerF3A*)Global::gameHandler;
      aHandler->set_draw_indicators(indicators);
      aHandler->set_draw_trajectory(trajectory);
      aHandler->set_draw_grid(grid);
      aHandler->set_tolerance(tolerance);
      aHandler->set_persistence(persistence);
      aHandler->set_grid_size(size);
      aHandler->set_flight_height(height);
      aHandler->set_flight_distance(distance);
      aHandler->set_orientation(orientation);
      aHandler->set_position_n(position_n);
      aHandler->set_position_e(position_e);
    }
  }
  puDeleteObject(obj);
}
