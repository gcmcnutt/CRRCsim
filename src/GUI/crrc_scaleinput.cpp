/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project

 * Copyright (C) 2005, 2008 Jens Wilhelm Wulf (original author)
 * Copyright (C) 2005, 2008 Jan Reucker
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
 *  \file crrc_scaleinput.cpp
 *  Implementation of class CGUIMixerDialog
 */

#include "../i18n.h"
#include "crrc_scaleinput.h"

#include <stdio.h>
#include <stdlib.h>

#include "util.h"
#include "../global.h"
#include "../aircraft.h"
#include "../mod_fdm/fdm.h"
#include "../config.h"
#include "../crrc_main.h"
#include "../mod_misc/lib_conversions.h"
#include "../mod_misc/SimpleXMLTransfer.h"

static void CGUIMixerCallback(puObject *obj);
static void CGUIMixerPresetCallback(puObject *obj);
static void CGUIMixerNewPresetCallback(puObject *obj);
static void CGUIMixerEnableButtonCallback(puObject *obj);

#define BUTTON_BOX_HEIGHT   (2*DLG_DEF_SPACE+DLG_DEF_BUTTON_HEIGHT)
#define SLIDER_W            (110)
#define SLIDER_H            (DLG_DEF_BUTTON_HEIGHT)
#define COMBO_W             (330)
#define COMBO_H             (DLG_DEF_BUTTON_HEIGHT)
#define SLIDER_LABEL_W      (80)
#define COMBO_LABEL_W       (250)
#define NUM_W               (60)

CGUIMixerDialog::CGUIMixerDialog(T_TX_Interface* itxi)
  : CRRCDialog()
{
  txi = itxi;
  
  // Load presets
    // first copy those from the config file
  presetGrp = new SimpleXMLTransfer(cfgfile->getChild("presets.mixer",true));

    // now add those from the current airplane
  SimpleXMLTransfer *alp = Global::aircraft->getFDMInterface()->getMixerPresets();
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

  // Create Widgets (construct in the inverse order of their display)
  int y_in_win = BUTTON_BOX_HEIGHT;
  
    // save preset
  inputNewName = new puInput(DLG_DEF_SPACE,           y_in_win,
                             COMBO_W + DLG_DEF_SPACE, y_in_win + SLIDER_H);
  inputNewName->setValue(_("Name of new preset"));
  puOneShot* buttonTmp = new puOneShot(COMBO_W + DLG_DEF_SPACE,                 y_in_win,
                                       COMBO_W + DLG_DEF_SPACE + COMBO_LABEL_W, y_in_win + SLIDER_H);
  y_in_win += ( SLIDER_H + DLG_DEF_SPACE );                              
  buttonTmp->setLegend(_("Save as new preset"));
  buttonTmp->setCallback(CGUIMixerNewPresetCallback);
  buttonTmp->setUserData(this); 

    // a group containing everything but the "enable" button
  all_widgets = new puGroup(0, 0);
  
  puText* info = new puText(DLG_DEF_SPACE, y_in_win + SLIDER_H/2);
  y_in_win += ( DLG_DEF_BUTTON_HEIGHT + DLG_DEF_SPACE );                              
  info->setLabelPlace(PUPLACE_CENTERED_RIGHT);
  info->setLabel(_("Hit tab or return after editing a value. Also see documentation/options.txt"));

    // mixers widgets
  for (int n=NrOfMixers-1; n >=0; n--)
  {
    mix_enable_button[n] = new puButton(0*DLG_DEF_SPACE + SLIDER_LABEL_W + DLG_DEF_SPACE + 0*SLIDER_W,               y_in_win,
                                        0*DLG_DEF_SPACE + SLIDER_LABEL_W + DLG_DEF_SPACE + 0*SLIDER_W + DLG_CHECK_W, y_in_win + DLG_CHECK_H);
    mix_enable_button[n]->setLabel(Global::inputDev->MixerStringsGUI[n]);
    mix_enable_button[n]->reveal();
    mix_enable_button[n]->setLabelPlace(PUPLACE_CENTERED_LEFT);
    mix_enable_button[n]->setUserData(this);
    mix_enable_button[n]->setButtonType(PUBUTTON_VCHECK);
    mix_enable_button[n]->setValue(0);
    if (txi->mixer->mixer_enabled[n])
    {
      mix_enable_button[n]->setValue(1);
    }

    combo_mix_src[n] = new puaComboBox(1*DLG_DEF_SPACE + SLIDER_LABEL_W + DLG_DEF_SPACE + 1*SLIDER_W,            y_in_win,
                                       1*DLG_DEF_SPACE + SLIDER_LABEL_W + DLG_DEF_SPACE + 1*SLIDER_W + SLIDER_W, y_in_win + SLIDER_H,
                                       NULL, false);
    combo_mix_src[n]->setChildColourScheme(PUCLASS_POPUPMENU, dlgCol1[0], dlgCol1[1], dlgCol1[2]);
    combo_mix_src[n]->newList(Global::inputDev->AxisStringsGUI);
    combo_mix_src[n]->setLabelPlace(PUPLACE_CENTERED_LEFT);
    combo_mix_src[n]->setLabel("");
    combo_mix_src[n]->setCurrentItem(txi->mixer->mixer_src[n]);
    combo_mix_src[n]->reveal();
    combo_mix_src[n]->activate();
    combo_mix_src[n]->setUserData(this);

    slider_mix_val[n] = new crrcSlider(2*DLG_DEF_SPACE + SLIDER_LABEL_W + DLG_DEF_SPACE + 2*SLIDER_W, y_in_win,
                                       2*DLG_DEF_SPACE + SLIDER_LABEL_W + DLG_DEF_SPACE + 3*SLIDER_W, y_in_win + SLIDER_H,
                                       NUM_W);
    slider_mix_val[n]->setLabelPlace(PUPLACE_CENTERED_LEFT);
    slider_mix_val[n]->setLabel("");
    slider_mix_val[n]->setSliderFraction(0.2);
    slider_mix_val[n]->setMinValue(-2.01);
    slider_mix_val[n]->setMaxValue(2.0);
    slider_mix_val[n]->setStepSize(0.05);
    slider_mix_val[n]->setValue(txi->mixer->mixer_val[n]);

    combo_mix_dst[n] = new puaComboBox(3*DLG_DEF_SPACE + SLIDER_LABEL_W + DLG_DEF_SPACE + 3*SLIDER_W,            y_in_win,
                                       3*DLG_DEF_SPACE + SLIDER_LABEL_W + DLG_DEF_SPACE + 3*SLIDER_W + SLIDER_W, y_in_win + SLIDER_H,
                                       NULL, false);
    y_in_win += ( SLIDER_H + DLG_DEF_SPACE );                              
    combo_mix_dst[n]->setChildColourScheme(PUCLASS_POPUPMENU, dlgCol1[0], dlgCol1[1], dlgCol1[2]);
    combo_mix_dst[n]->newList(Global::inputDev->AxisStringsGUI);
    combo_mix_dst[n]->setLabelPlace(PUPLACE_CENTERED_LEFT);
    combo_mix_dst[n]->setLabel("");
    combo_mix_dst[n]->setCurrentItem(txi->mixer->mixer_dst[n]);
    combo_mix_dst[n]->reveal();
    combo_mix_dst[n]->activate();
    combo_mix_dst[n]->setUserData(this);
  }
  
  y_in_win -= ( DLG_DEF_SPACE );                              
  for (int n=0; n<4; n++)
  {
    mixer_labels[n] = new puText(DLG_DEF_SPACE + SLIDER_LABEL_W + n*(DLG_DEF_SPACE + SLIDER_W), y_in_win);
  }
  y_in_win += ( DLG_DEF_BUTTON_HEIGHT + DLG_DEF_SPACE );                              
  mixer_labels[0]->setLabel(_("Enable"));
  mixer_labels[1]->setLabel(_("Source"));
  mixer_labels[2]->setLabel(_("Mixing rate"));
  mixer_labels[3]->setLabel(_("Destination"));
    
    // axis widgets
  for (int n=NrOfAxes-1; n>=0; n--)
  {
    slider_trim[n] = new crrcSlider(0*DLG_DEF_SPACE + SLIDER_LABEL_W + DLG_DEF_SPACE + 0*SLIDER_W, y_in_win,
                                    0*DLG_DEF_SPACE + SLIDER_LABEL_W + DLG_DEF_SPACE + 1*SLIDER_W, y_in_win + SLIDER_H,
                                    NUM_W);
    slider_trim[n]->setLabelPlace(PUPLACE_CENTERED_LEFT);
    slider_trim[n]->setLabel("");
    slider_trim[n]->setSliderFraction(0.2);
    slider_trim[n]->setMinValue(-0.5);
    slider_trim[n]->setMaxValue(0.5);
    slider_trim[n]->setStepSize(0.02);

    if (n <= 2)
    {
      // normal rate setting for aileron, elevator and rudder
      slider_nrate[n] = new crrcSlider(1*DLG_DEF_SPACE + SLIDER_LABEL_W + DLG_DEF_SPACE + 1*SLIDER_W, y_in_win,
                                       1*DLG_DEF_SPACE + SLIDER_LABEL_W + DLG_DEF_SPACE + 2*SLIDER_W, y_in_win + SLIDER_H,
                                       NUM_W);
      slider_nrate[n]->setLabelPlace(PUPLACE_CENTERED_LEFT);
      slider_nrate[n]->setLabel("");
      slider_nrate[n]->setSliderFraction(0.2);
      slider_nrate[n]->setMinValue(0.0);
      slider_nrate[n]->setMaxValue(1.0);
      slider_nrate[n]->setStepSize(0.02);

      // slow rate setting for aileron, elevator and rudder
      slider_srate[n] = new crrcSlider(2*DLG_DEF_SPACE + SLIDER_LABEL_W + DLG_DEF_SPACE + 2*SLIDER_W, y_in_win,
                                       2*DLG_DEF_SPACE + SLIDER_LABEL_W + DLG_DEF_SPACE + 3*SLIDER_W, y_in_win + SLIDER_H,
                                       NUM_W);
      slider_srate[n]->setLabelPlace(PUPLACE_CENTERED_LEFT);
      slider_srate[n]->setLabel("");
      slider_srate[n]->setSliderFraction(0.2);
      slider_srate[n]->setMinValue(0.0);
      slider_srate[n]->setMaxValue(1.0);
      slider_srate[n]->setStepSize(0.02);

      // exp setting for aileron, elevator and rudder
      slider_exp[n] = new crrcSlider(3*DLG_DEF_SPACE + SLIDER_LABEL_W + DLG_DEF_SPACE + 3*SLIDER_W, y_in_win,
                                     3*DLG_DEF_SPACE + SLIDER_LABEL_W + DLG_DEF_SPACE + 4*SLIDER_W, y_in_win + SLIDER_H,
                                     NUM_W);
      slider_exp[n]->setLabelPlace(PUPLACE_CENTERED_LEFT);
      slider_exp[n]->setLabel("");
      slider_exp[n]->setSliderFraction(0.2);
      slider_exp[n]->setMinValue(0.0);
      slider_exp[n]->setMaxValue(1.0);
      slider_exp[n]->setStepSize(0.02);
    }

    if (n == 4)
    {
      // - travel setting for flap
      slider_nrate[n] = new crrcSlider( 1*DLG_DEF_SPACE + SLIDER_LABEL_W + DLG_DEF_SPACE + (int)(1.5*SLIDER_W), y_in_win,
                                        1*DLG_DEF_SPACE + SLIDER_LABEL_W + DLG_DEF_SPACE + (int)(2.5*SLIDER_W), y_in_win + SLIDER_H,
                                        NUM_W);
      slider_nrate[n]->setLabelPlace(PUPLACE_CENTERED_LEFT);
      slider_nrate[n]->setLabel(_("-Travel"));
      slider_nrate[n]->setSliderFraction(0.2);
      slider_nrate[n]->setMinValue(-0.5);
      slider_nrate[n]->setMaxValue(0.0);
      slider_nrate[n]->setStepSize(0.02);

      // + travel setting for flap
      slider_srate[n] = new crrcSlider( 2*DLG_DEF_SPACE + SLIDER_LABEL_W + DLG_DEF_SPACE + (int)(2.5*SLIDER_W), y_in_win,
                                        2*DLG_DEF_SPACE + SLIDER_LABEL_W + DLG_DEF_SPACE + (int)(3.5*SLIDER_W), y_in_win + SLIDER_H,
                                        NUM_W);
      slider_srate[n]->setLabelPlace(PUPLACE_CENTERED_RIGHT);
      slider_srate[n]->setLabel(_("+Travel"));
      slider_srate[n]->setSliderFraction(0.2);
      slider_srate[n]->setMinValue(0.0);
      slider_srate[n]->setMaxValue(0.5);
      slider_srate[n]->setStepSize(0.02);
    }
    y_in_win += ( SLIDER_H + DLG_DEF_SPACE );                              
    
    switch (n)
    {
     case 0:
      slider_trim[n]  ->setLabel(_("Aileron"));
      slider_trim[n]  ->setValue(txi->mixer->trim_val[T_AxisMapper::AILERON]); 
      slider_nrate[n] ->setValue(txi->mixer->nrate_val[T_AxisMapper::AILERON]);       
      slider_srate[n] ->setValue(txi->mixer->srate_val[T_AxisMapper::AILERON]);       
      slider_exp[n]   ->setValue(txi->mixer->exp_val[T_AxisMapper::AILERON]); 
      break;     
     case 1:
      slider_trim[n]  ->setLabel(_("Elevator"));
      slider_trim[n]  ->setValue(txi->mixer->trim_val[T_AxisMapper::ELEVATOR]); 
      slider_nrate[n] ->setValue(txi->mixer->nrate_val[T_AxisMapper::ELEVATOR]);
      slider_srate[n] ->setValue(txi->mixer->srate_val[T_AxisMapper::ELEVATOR]);
      slider_exp[n]   ->setValue(txi->mixer->exp_val[T_AxisMapper::ELEVATOR]);
      break;      
     case 2:
      slider_trim[n]  ->setLabel(_("Rudder"));
      slider_trim[n]  ->setValue(txi->mixer->trim_val[T_AxisMapper::RUDDER]); 
      slider_nrate[n] ->setValue(txi->mixer->nrate_val[T_AxisMapper::RUDDER]);
      slider_srate[n] ->setValue(txi->mixer->srate_val[T_AxisMapper::RUDDER]);
      slider_exp[n]   ->setValue(txi->mixer->exp_val[T_AxisMapper::RUDDER]);
      break;      
     case 3:
      slider_trim[n]  ->setLabel(_("Throttle"));
      slider_trim[n]  ->setValue(txi->mixer->trim_val[T_AxisMapper::THROTTLE]); 
      break;      
     case 4:
      slider_trim[n]  ->setLabel(_("Flap"));
      slider_trim[n]  ->setValue(txi->mixer->trim_val[T_AxisMapper::FLAP]); 
      slider_nrate[n] ->setValue(txi->mixer->mtravel_val[T_AxisMapper::FLAP]);
      slider_srate[n] ->setValue(txi->mixer->ptravel_val[T_AxisMapper::FLAP]);
      break;      
     default:
      slider_trim[n]  ->setLabel(_("Spoiler"));
      slider_trim[n]  ->setValue(txi->mixer->trim_val[T_AxisMapper::SPOILER]);
      break;      
    }
  }
  
  y_in_win -= ( DLG_DEF_SPACE );                              
  for (int n=4-1; n>=0; n--)
  {
    labels[n] = new puText(DLG_DEF_SPACE + SLIDER_LABEL_W + n*(DLG_DEF_SPACE + SLIDER_W),  y_in_win);
  }
  y_in_win += ( DLG_DEF_BUTTON_HEIGHT + DLG_DEF_SPACE );                              
  labels[0]->setLabel(_("Trim/Offset"));
  labels[1]->setLabel(_("Normal rate"));
  labels[2]->setLabel(_("Slow rate"));
  labels[3]->setLabel(_("Exp"));
  
    // dual-rate enable widget
  dr_enable_button = new puButton(DLG_DEF_SPACE,                y_in_win,
                                  DLG_DEF_SPACE + DLG_CHECK_W,  y_in_win + DLG_CHECK_H);
  y_in_win += ( DLG_CHECK_H + DLG_DEF_SPACE );                               
  dr_enable_button->setLabel("Dual-rate enable");
  dr_enable_button->reveal();
  dr_enable_button->setLabelPlace(PUPLACE_CENTERED_RIGHT);
  dr_enable_button->setUserData(this);
  dr_enable_button->setButtonType(PUBUTTON_VCHECK);
  dr_enable_button->setValue(0);
  if (txi->mixer->dr_enabled)
  {
    dr_enable_button->setValue(1);
  }
  
  all_widgets->close();

  if (txi->mixer->enabled)
  {
    all_widgets->reveal();
  }
  else
  {
    all_widgets->hide();
  }
  
    // mixer enable button
  enable_button = new puButton( DLG_DEF_SPACE,                y_in_win,
                                DLG_DEF_SPACE + DLG_CHECK_W,  y_in_win + DLG_CHECK_H);
  y_in_win += ( DLG_CHECK_H + DLG_DEF_SPACE );                               
  enable_button->setLabel(_("Enable"));
  enable_button->reveal();
  enable_button->setLabelPlace(PUPLACE_CENTERED_RIGHT);
  enable_button->setUserData(this);
  enable_button->setButtonType(PUBUTTON_VCHECK);
  enable_button->setValue(0);
  enable_button->setCallback(CGUIMixerEnableButtonCallback);
  if (txi->mixer->enabled)
  {
    enable_button->setValue(1);
  }

    // preset choice
  comboPresets = new puaComboBox(COMBO_LABEL_W + DLG_DEF_SPACE, y_in_win,
                                 COMBO_LABEL_W + COMBO_W,       y_in_win + SLIDER_H,
                                 NULL, false);
  y_in_win += ( SLIDER_H + DLG_DEF_SPACE );                               
  comboPresets->setChildColourScheme(PUCLASS_POPUPMENU, dlgCol1[0], dlgCol1[1], dlgCol1[2]);
  comboPresets->newList(presets);
  comboPresets->setLabelPlace(PUPLACE_CENTERED_LEFT);
  comboPresets->setLabel(_("Load Preset"));
  comboPresets->setCurrentItem(-1);
  comboPresets->setCallback(CGUIMixerPresetCallback);
  comboPresets->reveal();
  comboPresets->activate();
  comboPresets->setUserData(this);

  // finalize the dialog
  close();
  // keep largest width
  //setSize(3*DLG_DEF_SPACE + 4*SLIDER_W + 1*SLIDER_LABEL_W + 2*DLG_DEF_SPACE, y_in_win);
  setSize(2*DLG_DEF_SPACE + COMBO_LABEL_W + COMBO_W, y_in_win);
  setCallback(CGUIMixerCallback);
  centerOnScreen();
  reveal();
}

/**
 * Destroy the dialog.
 */
CGUIMixerDialog::~CGUIMixerDialog()
{
  T_GUI_Util::freenames(presets, nPresets);
}

/** \brief callback to load a preset.
 *
 */
void CGUIMixerPresetCallback(puObject *obj)
{
  CGUIMixerDialog* dlg   = (CGUIMixerDialog*)obj->getUserData();
  int              nItem = dlg->comboPresets->getCurrentItem();

  if (nItem == 0)
  {
    dlg->inputNewName->setValue("Name of new preset");

    // the "default" entry
    dlg->enable_button   ->setValue(1);
    dlg->dr_enable_button->setValue(0);

    dlg->slider_trim[0]  ->setValue((float)0.0);
    dlg->slider_nrate[0] ->setValue((float)1.0);
    dlg->slider_srate[0] ->setValue((float)1.0);
    dlg->slider_exp[0]   ->setValue((float)0.0);
    
    dlg->slider_trim[1]  ->setValue((float)0.0);
    dlg->slider_nrate[1] ->setValue((float)1.0);
    dlg->slider_srate[1] ->setValue((float)1.0);
    dlg->slider_exp[1]   ->setValue((float)0.0);
    
    dlg->slider_trim[2]  ->setValue((float)0.0);
    dlg->slider_nrate[2] ->setValue((float)1.0);
    dlg->slider_srate[2] ->setValue((float)1.0);
    dlg->slider_exp[2]   ->setValue((float)0.0);
    
    dlg->slider_trim[3]  ->setValue((float)0.0);
    
    dlg->slider_trim[4]  ->setValue((float)0.0);
    dlg->slider_nrate[4] ->setValue((float)-0.5);
    dlg->slider_srate[4] ->setValue((float)0.5);
    
    dlg->slider_trim[5]  ->setValue((float)0.0);

    for (int i=0; i<dlg->NrOfMixers; i++)
    {
      dlg->mix_enable_button[i] ->setValue(0);
      dlg->combo_mix_src[i]     ->setCurrentItem(T_AxisMapper::NOTHING);
      dlg->combo_mix_dst[i]     ->setCurrentItem(T_AxisMapper::NOTHING);    
      dlg->slider_mix_val[i]    ->setValue((float)0.0);
    }
  }
  else
  {
    SimpleXMLTransfer* preset = dlg->presetGrp->getChildAt(nItem-1);
    SimpleXMLTransfer* item;
       
    dlg->enable_button   ->setValue(preset->getInt("enabled"));
    dlg->dr_enable_button->setValue(preset->getInt("dr_enabled"));

    item = preset->getChild(Global::inputDev->AxisStringsXML[T_AxisMapper::AILERON], true);
    dlg->slider_trim[0]  ->setValue((float)item->getDouble("trim"));
    dlg->slider_nrate[0] ->setValue((float)item->getDouble("nrate"));
    dlg->slider_srate[0] ->setValue((float)item->getDouble("srate"));
    dlg->slider_exp[0]   ->setValue((float)item->getDouble("exp"));
    
    item = preset->getChild(Global::inputDev->AxisStringsXML[T_AxisMapper::ELEVATOR], true);
    dlg->slider_trim[1]  ->setValue((float)item->getDouble("trim"));
    dlg->slider_nrate[1] ->setValue((float)item->getDouble("nrate"));
    dlg->slider_srate[1] ->setValue((float)item->getDouble("srate"));
    dlg->slider_exp[1]   ->setValue((float)item->getDouble("exp"));
    
    item = preset->getChild(Global::inputDev->AxisStringsXML[T_AxisMapper::RUDDER], true);
    dlg->slider_trim[2]  ->setValue((float)item->getDouble("trim"));
    dlg->slider_nrate[2] ->setValue((float)item->getDouble("nrate"));
    dlg->slider_srate[2] ->setValue((float)item->getDouble("srate"));
    dlg->slider_exp[2]   ->setValue((float)item->getDouble("exp"));
    
    item = preset->getChild(Global::inputDev->AxisStringsXML[T_AxisMapper::THROTTLE], true);
    dlg->slider_trim[3]  ->setValue((float)item->getDouble("trim"));
    
    item = preset->getChild(Global::inputDev->AxisStringsXML[T_AxisMapper::FLAP], true);
    dlg->slider_trim[4]  ->setValue((float)item->getDouble("trim"));
    dlg->slider_nrate[4] ->setValue((float)item->getDouble("mtravel"));
    dlg->slider_srate[4] ->setValue((float)item->getDouble("ptravel"));
    
    item = preset->getChild(Global::inputDev->AxisStringsXML[T_AxisMapper::SPOILER], true);
    dlg->slider_trim[5]  ->setValue((float)item->getDouble("trim"));

    for (int i=0; i<dlg->NrOfMixers; i++)
    {
      item = preset->getChild(Global::inputDev->MixerStringsXML[i], true);
      dlg->mix_enable_button[i] ->setValue(item->getInt("enabled"));
      dlg->combo_mix_src[i]     ->setCurrentItem(item->getInt("src"));
      dlg->combo_mix_dst[i]     ->setCurrentItem(item->getInt("dst"));    
      dlg->slider_mix_val[i]    ->setValue((float)item->getDouble("val"));
    }
  }

  if (dlg->enable_button->getIntegerValue())
  {
    dlg->all_widgets->reveal();
  }
  else
  {
    dlg->all_widgets->hide();
  }
}

 /** \brief callback to save a preset.
 *
 */
void CGUIMixerNewPresetCallback(puObject *obj)
{
  CGUIMixerDialog* dlg   = (CGUIMixerDialog*)obj->getUserData();

  SimpleXMLTransfer* mixer   = new SimpleXMLTransfer();

  mixer->setName("mixer");
  mixer->setAttribute("name_en",    dlg->inputNewName->getStringValue());  
  mixer->setAttribute("enabled",    dlg->enable_button->getIntegerValue());
  mixer->setAttribute("dr_enabled", dlg->dr_enable_button->getIntegerValue());
  
  SimpleXMLTransfer* item;

  item = mixer->getChild(Global::inputDev->AxisStringsXML[T_AxisMapper::AILERON], true);
  item->setAttribute("trim",  doubleToString(dlg->slider_trim[0]  ->getFloatValue()));
  item->setAttribute("nrate", doubleToString(dlg->slider_nrate[0] ->getFloatValue()));
  item->setAttribute("srate", doubleToString(dlg->slider_srate[0] ->getFloatValue()));
  item->setAttribute("exp",   doubleToString(dlg->slider_exp[0]   ->getFloatValue()));

  item = mixer->getChild(Global::inputDev->AxisStringsXML[T_AxisMapper::ELEVATOR], true);
  item->setAttribute("trim",  doubleToString(dlg->slider_trim[1]  ->getFloatValue()));
  item->setAttribute("nrate", doubleToString(dlg->slider_nrate[1] ->getFloatValue()));
  item->setAttribute("srate", doubleToString(dlg->slider_srate[1] ->getFloatValue()));
  item->setAttribute("exp",   doubleToString(dlg->slider_exp[1]   ->getFloatValue()));

  item = mixer->getChild(Global::inputDev->AxisStringsXML[T_AxisMapper::RUDDER], true);
  item->setAttribute("trim",  doubleToString(dlg->slider_trim[2]  ->getFloatValue()));
  item->setAttribute("nrate", doubleToString(dlg->slider_nrate[2] ->getFloatValue()));
  item->setAttribute("srate", doubleToString(dlg->slider_srate[2] ->getFloatValue()));
  item->setAttribute("exp",   doubleToString(dlg->slider_exp[2]   ->getFloatValue()));

  item = mixer->getChild(Global::inputDev->AxisStringsXML[T_AxisMapper::THROTTLE], true);
  item->setAttribute("trim",  doubleToString(dlg->slider_trim[3]  ->getFloatValue()));

  item = mixer->getChild(Global::inputDev->AxisStringsXML[T_AxisMapper::FLAP], true);
  item->setAttribute("trim",    doubleToString(dlg->slider_trim[4]  ->getFloatValue()));
  item->setAttribute("mtravel", doubleToString(dlg->slider_nrate[4] ->getFloatValue()));
  item->setAttribute("ptravel", doubleToString(dlg->slider_srate[4] ->getFloatValue()));
  
  item = mixer->getChild(Global::inputDev->AxisStringsXML[T_AxisMapper::SPOILER], true);
  item->setAttribute("trim",  doubleToString(dlg->slider_trim[5]  ->getFloatValue()));

  for (int i=0; i<dlg->NrOfMixers; i++)
  {
    item = mixer->getChild(Global::inputDev->MixerStringsXML[i], true);
    item->setAttribute("enabled", dlg->mix_enable_button[i]->getIntegerValue());
    item->setAttribute("src",     dlg->combo_mix_src[i]->getCurrentItem());
    item->setAttribute("dst",     dlg->combo_mix_dst[i]->getCurrentItem());
    item->setAttribute("val",     doubleToString(dlg->slider_mix_val[i]->getFloatValue()));
  }
  
  cfgfile->getChild("presets.mixer", true)->addChild(mixer);
  
  // save file
  options_saveToFile();
  
  // exit dialog (otherwise we would have to regenerate presets)
  dlg->setValue(CRRC_DIALOG_OK);
  CGUIMixerCallback(dlg);
}

/** \brief The dialog's callback.
 *
 */
void CGUIMixerCallback(puObject *obj)
{
  CGUIMixerDialog* dlg   = (CGUIMixerDialog*)obj;

  if (obj->getIntegerValue() == CRRC_DIALOG_OK)
  {
    // Dialog left by clicking OK
    dlg->txi->mixer->enabled    = dlg->enable_button->getIntegerValue();
    dlg->txi->mixer->dr_enabled = dlg->dr_enable_button->getIntegerValue();

    dlg->txi->mixer->trim_val[T_AxisMapper::AILERON]   = dlg->slider_trim[0]  ->getFloatValue();
    dlg->txi->mixer->nrate_val[T_AxisMapper::AILERON]  = dlg->slider_nrate[0] ->getFloatValue();
    dlg->txi->mixer->srate_val[T_AxisMapper::AILERON]  = dlg->slider_srate[0] ->getFloatValue();
    dlg->txi->mixer->exp_val[T_AxisMapper::AILERON]    = dlg->slider_exp[0]   ->getFloatValue();
    
    dlg->txi->mixer->trim_val[T_AxisMapper::ELEVATOR]  = dlg->slider_trim[1]  ->getFloatValue();
    dlg->txi->mixer->nrate_val[T_AxisMapper::ELEVATOR] = dlg->slider_nrate[1] ->getFloatValue();
    dlg->txi->mixer->srate_val[T_AxisMapper::ELEVATOR] = dlg->slider_srate[1] ->getFloatValue();
    dlg->txi->mixer->exp_val[T_AxisMapper::ELEVATOR]   = dlg->slider_exp[1]   ->getFloatValue();
    
    dlg->txi->mixer->trim_val[T_AxisMapper::RUDDER]    = dlg->slider_trim[2]  ->getFloatValue();
    dlg->txi->mixer->nrate_val[T_AxisMapper::RUDDER]   = dlg->slider_nrate[2] ->getFloatValue();
    dlg->txi->mixer->srate_val[T_AxisMapper::RUDDER]   = dlg->slider_srate[2] ->getFloatValue();
    dlg->txi->mixer->exp_val[T_AxisMapper::RUDDER]     = dlg->slider_exp[2]   ->getFloatValue();
    
    dlg->txi->mixer->trim_val[T_AxisMapper::THROTTLE]  = dlg->slider_trim[3]  ->getFloatValue();
    
    dlg->txi->mixer->trim_val[T_AxisMapper::FLAP]      = dlg->slider_trim[4]  ->getFloatValue();
    dlg->txi->mixer->mtravel_val[T_AxisMapper::FLAP]   = dlg->slider_nrate[4] ->getFloatValue();
    dlg->txi->mixer->ptravel_val[T_AxisMapper::FLAP]   = dlg->slider_srate[4] ->getFloatValue();
    
    dlg->txi->mixer->trim_val[T_AxisMapper::SPOILER]   = dlg->slider_trim[5]  ->getFloatValue();

    for (int i=0; i<dlg->NrOfMixers; i++)
    {
      dlg->txi->mixer->mixer_enabled[i] = dlg->mix_enable_button[i]->getIntegerValue();
      dlg->txi->mixer->mixer_src[i]     = dlg->combo_mix_src[i]->getCurrentItem();
      dlg->txi->mixer->mixer_val[i]     = dlg->slider_mix_val[i]->getFloatValue();
      dlg->txi->mixer->mixer_dst[i]     = dlg->combo_mix_dst[i]->getCurrentItem();    
    }
    
    // Save new settings
    dlg->txi->mixer->putBackIntoCfg(cfgfile);
  }
  delete dlg->presetGrp;
  puDeleteObject(obj);
}

/** 
 *  Callback for the "enable" button.
 */
void CGUIMixerEnableButtonCallback(puObject *obj)
{
  CGUIMixerDialog* dlg   = (CGUIMixerDialog*)obj->getUserData();
  
  if (obj->getIntegerValue())
  {
    dlg->all_widgets->reveal();
  }
  else
  {
    dlg->all_widgets->hide();
  }
}
