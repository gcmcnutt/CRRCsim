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
  

// F3F dialogs
#include "../i18n.h"
#include "crrc_f3f.h"

#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>

#include "../global.h"
#include "util.h"
#include "../crrc_main.h"
#include "../mod_misc/lib_conversions.h"
#include "../mod_misc/SimpleXMLTransfer.h"
#include "../mod_mode/T_GameHandler.h"
#include "../mod_mode/F3F/handlerF3F.h"

static void CGUIF3FDialogCallback(puObject * obj);


#define BUTTON_BOX_HEIGHT   (2*DLG_DEF_SPACE+DLG_DEF_BUTTON_HEIGHT)
#define SLIDER_W            308
#define SLIDER_H            DLG_DEF_BUTTON_HEIGHT
#define COMBO_H             DLG_DEF_BUTTON_HEIGHT
#define COMBO_W             330
#define LABEL_W             250
#define NUM_W               70


CGUIF3FDialog::CGUIF3FDialog():CRRCDialog()
{
  int f3f_cfg_enabled;
  int y_in_win = 0;

  // assure that all attributes exist in the config file
  HandlerF3F::prepareConfigFile(cfgfile);
  SimpleXMLTransfer* f3fcfg = cfg->getCurLocCfgPtr(cfgfile)->getChild("game.f3f", true);

  
  f3f_cfg_enabled = cfgfile->getInt("game.f3f.enabled");
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
  //buttonTmp->setCallback(CGUIF3FDialogNewPresetCallback);
  //buttonTmp->setUserData(this);      

  // select sound files

  std::vector<std::string> dirs;
  T_Config::getF3FSoundDirs(dirs);
  dirs.push_back(std::string("beep"));
  unsigned int last_dir = 0;
  
  // The sound folder combo box shall reflect the current setting from
  // the config file, so we'll compare the configured dirname to all
  // dirs while constructing the combo box entry list.
  // The default folder is sounds/f3f/default. This folder exists when
  // running directly from the build directory or in a Windows environment
  // where everything is installed in one place. In a Linux environment,
  // the data files are usually installed separately, somewhere below
  // the "share" directory (e.g. /usr/local/share). So besides looking
  // for an exact match we should check for a "good" match that contains
  // sounds/f3f/default as a substring.
  std::string config_sound_dir = cfgfile->getString("game.f3f.sound.dir");
  int sound_index = -1;
  int good_sound_index = -1;

  for (std::vector<std::string>::size_type index = 0; index < dirs.size(); index++)
  {
    if (index < MAX_F3F_SOUND_DIRS)
    {
      sound_list[index] = new char[dirs[index].length() + 1];
      strcpy(sound_list[index], dirs[index].c_str());
      last_dir = index;
      
      // find sound folder from config file, check for exact match first
      if ((sound_index < 0) && (dirs[index] == config_sound_dir))
      {
        // exact match!
        sound_index = index;
      }
      else if ((good_sound_index < 0) 
                && 
               (dirs[index].rfind(config_sound_dir) != std::string::npos))
      {
        // good match
        good_sound_index = index;
      }
    }
  }
  last_dir++;
  sound_list[last_dir] = NULL;
  
  if (sound_index < 0)
  {
    if (good_sound_index < 0)
    {
      // neither good nor exact match
      sound_index = 0;
    }
    else
    {
      // only good match found
      sound_index = good_sound_index;
    }
  }

  soundSelectBox = new puaComboBox(LABEL_W + DLG_DEF_SPACE, y_in_win,
                                  LABEL_W + COMBO_W, y_in_win + COMBO_H,
                                  NULL, false);
  y_in_win += COMBO_H + DLG_DEF_SPACE;
  soundSelectBox->setChildColourScheme(PUCLASS_POPUPMENU, dlgCol1[0], dlgCol1[1], dlgCol1[2]);
  soundSelectBox->newList(sound_list);
  soundSelectBox->setLabelPlace(PUPLACE_LOWER_LEFT);
  soundSelectBox->setLabel(_("Sounds file directory"));
  soundSelectBox->setCurrentItem(sound_index);
  //soundSelectBox ->setCallback(CGUIF3FSoundSelCallback);
  soundSelectBox->reveal();
  soundSelectBox->activate();
  soundSelectBox->setUserData(this);

  // ------------ extend bases
  f3f_extend_bases = new puButton(LABEL_W + DLG_DEF_SPACE, y_in_win,
                                  LABEL_W + DLG_DEF_SPACE + DLG_CHECK_W,
                                  y_in_win + DLG_CHECK_H);
  y_in_win += DLG_CHECK_H + DLG_DEF_SPACE;
  f3f_extend_bases->setLabelPlace(PUPLACE_CENTERED_LEFT);
  f3f_extend_bases->setLabel(_("Show bases planes"));
  f3f_extend_bases->setButtonType(PUBUTTON_VCHECK);
  f3f_extend_bases->setValue(cfgfile->getInt("game.f3f.extend_bases"));

  // ------------ start on left
  start_on_left = new puButton(LABEL_W + DLG_DEF_SPACE, y_in_win,
                               LABEL_W + DLG_DEF_SPACE + DLG_CHECK_W,
                               y_in_win + DLG_CHECK_H);
  y_in_win += DLG_CHECK_H + DLG_DEF_SPACE;
  start_on_left->setLabelPlace(PUPLACE_CENTERED_LEFT);
  start_on_left->setLabel(_("Start on the left"));
  start_on_left->setButtonType(PUBUTTON_VCHECK);
  start_on_left->setValue(cfgfile->getInt("game.f3f.start_left"));

  // ------------ security_line
  slider_security_line = new crrcSlider(LABEL_W + DLG_DEF_SPACE, y_in_win,
                                        LABEL_W + DLG_DEF_SPACE + SLIDER_W,
                                        y_in_win + SLIDER_H, NUM_W);
  y_in_win += SLIDER_H + DLG_DEF_SPACE;
  slider_security_line->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_security_line->setLabel(_("Security_line position [ft]"));
  slider_security_line->setSliderFraction(0.05);
  slider_security_line->setMinValue(-50);
  slider_security_line->setMaxValue(10);
  slider_security_line->setStepSize(4);
  slider_security_line->setValue(cfgfile->getInt("game.f3f.security_line"));

  // ------------ plan limit
  slider_bases_distance = new crrcSlider(LABEL_W + DLG_DEF_SPACE, y_in_win,
                                         LABEL_W + DLG_DEF_SPACE + SLIDER_W,
                                         y_in_win + SLIDER_H, NUM_W);
  y_in_win += SLIDER_H + DLG_DEF_SPACE;
  slider_bases_distance->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_bases_distance->setLabel(_("Distance between bases [ft]"));
  slider_bases_distance->setSliderFraction(0.05);
  slider_bases_distance->setMinValue(260);  // 260 = 80 m
  slider_bases_distance->setMaxValue(600);  // 390= 120 m
  slider_bases_distance->setStepSize(10);
  slider_bases_distance->setValue(f3fcfg->getInt("plan_limit"));
  
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
  slider_orientation->setValue(f3fcfg->getInt("orientation"));
  
   // ------------ position nord
  slider_position_n = new crrcSlider(LABEL_W + DLG_DEF_SPACE, y_in_win,
                                         LABEL_W + DLG_DEF_SPACE + SLIDER_W,
                                         y_in_win + SLIDER_H, NUM_W);
  y_in_win += SLIDER_H + DLG_DEF_SPACE;
  slider_position_n->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_position_n->setLabel(_("Center position Nord [ft]"));
  slider_position_n->setSliderFraction(0.05);
  slider_position_n->setMinValue(-100);  
  slider_position_n->setMaxValue(100);
  slider_position_n->setStepSize(1);
  slider_position_n->setValue(f3fcfg->getInt("position_north"));

   // ------------ position east
  slider_position_e = new crrcSlider(LABEL_W + DLG_DEF_SPACE, y_in_win,
                                         LABEL_W + DLG_DEF_SPACE + SLIDER_W,
                                         y_in_win + SLIDER_H, NUM_W);
  y_in_win += SLIDER_H + DLG_DEF_SPACE;
  slider_position_e->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_position_e->setLabel(_("Center position East [ft]"));
  slider_position_e->setSliderFraction(0.05);
  slider_position_e->setMinValue(-100);  
  slider_position_e->setMaxValue(100);
  slider_position_e->setStepSize(1);
  slider_position_e->setValue(f3fcfg->getInt("position_east"));

  // ------------ presets
  //  presetGrp = cfgfile->getChild("game.f3f.preset");    
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
  //  comboPresets->setCallback(CGUIF3FDialogPresetCallback);
  //  comboPresets->reveal();
  //  comboPresets->activate();
  //  comboPresets->setUserData(this);

  // ------------ enable F3F
  f3f_enable = new puButton(LABEL_W + DLG_DEF_SPACE, y_in_win,
                            LABEL_W + DLG_DEF_SPACE + DLG_CHECK_W,
                            y_in_win + DLG_CHECK_H);
  y_in_win += DLG_CHECK_H + DLG_DEF_SPACE;
  f3f_enable->setLabelPlace(PUPLACE_CENTERED_LEFT);
  f3f_enable->setLabel(_("Enable F3F mode"));
  f3f_enable->setButtonType(PUBUTTON_VCHECK);

  if (f3f_cfg_enabled == 1)
    f3f_enable->setValue(1);
  else
    f3f_enable->setValue(0);
  f3f_enable->reveal();

  //
  // sounds files
  // shadow mode
  // pilote position
  // zoom control
  // field of view

  // finalize the dialog
  close();
  setSize(COMBO_W + LABEL_W + 2 * DLG_DEF_SPACE, y_in_win);
  setCallback(CGUIF3FDialogCallback);
  centerOnScreen();
  reveal();
}


/**
 * Destroy the dialog.
 */

CGUIF3FDialog::~CGUIF3FDialog()
{
  int i = 0;
  while (sound_list[i] != NULL)
  {
    delete[] sound_list[i];
    i++;
  }
}


/** \brief The dialog's callback.
 *
 */
void CGUIF3FDialogCallback(puObject * obj)
{
  CGUIF3FDialog *dlg = (CGUIF3FDialog *) obj;

  if (obj->getIntegerValue() == CRRC_DIALOG_OK)
  {
    // create a new game handler depending on the "enabled" checkbox
    int f3f_is_enabled = dlg->f3f_enable->getIntegerValue();
    cfgfile->setAttributeOverwrite("game.f3f.enabled", f3f_is_enabled);
    if (f3f_is_enabled)
    {
      // if another game mode is currently active disable it
      if (Global::gameHandler->gameType() != std::string("F3F"))
        Global::gameHandler->disable();

      delete(Global::gameHandler);
      Global::gameHandler = new HandlerF3F();
    }
    else
    {
      // if this game mode is currently active disable it
      if (Global::gameHandler->gameType() == std::string("F3F"))
      {
        delete(Global::gameHandler);
        Global::gameHandler = new T_GameHandler();
      }
    }
    
    // in any case, save the values in the config file
    int sec_line = dlg->slider_security_line->getIntegerValue();
    cfgfile->setAttributeOverwrite("game.f3f.security_line", sec_line);
    int ext_bases = dlg->f3f_extend_bases->getIntegerValue();
    cfgfile->setAttributeOverwrite("game.f3f.extend_bases", ext_bases);
    int start_left = dlg->start_on_left->getIntegerValue();
    cfgfile->setAttributeOverwrite("game.f3f.start_left", start_left);
    int nItem = dlg->soundSelectBox->getCurrentItem();
    cfgfile->setAttributeOverwrite("game.f3f.sound.dir", dlg->sound_list[nItem]);
    //~ cfgfile->setAttributeOverwrite("game.f3f.sound.index", nItem);
    
    //save  location specifics parameters
    SimpleXMLTransfer* f3fcfg = cfg->getCurLocCfgPtr(cfgfile)->getChild("game.f3f", true);
    int plan_limit = dlg->slider_bases_distance->getIntegerValue();
    f3fcfg->setAttributeOverwrite("plan_limit", plan_limit);
    int orientation = dlg->slider_orientation->getIntegerValue();
    f3fcfg->setAttributeOverwrite("orientation", orientation);
    int position_n = dlg->slider_position_n->getIntegerValue();
    f3fcfg->setAttributeOverwrite("position_north", position_n);
    int position_e = dlg->slider_position_e->getIntegerValue();
    f3fcfg->setAttributeOverwrite("position_east", position_e);
    
    // if F3F is enabled, also update the running game handler
    // with the new configuration values
    if (Global::gameHandler->gameType() == std::string("F3F"))
    {
      HandlerF3F *aHandler = (HandlerF3F*)Global::gameHandler;
      aHandler->set_security_line(sec_line);
      aHandler->set_extend_bases(ext_bases);
      aHandler->set_start_left(start_left);
      aHandler->set_plan_limit(plan_limit);
      aHandler->set_position_n(position_n);
      aHandler->set_position_e(position_e);
      aHandler->set_orientation(orientation);
      aHandler->set_sound_dir(dlg->sound_list[nItem]);
    }
  }
  puDeleteObject(obj);
}
