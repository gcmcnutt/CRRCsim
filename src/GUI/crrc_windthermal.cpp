/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2006, 2008 Jens Wilhelm Wulf (original author)
 * Copyright (C) 2008 Jan Reucker
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
  

// implementation of class CGUIWindThermalDialog
//
#include "../i18n.h"
#include "crrc_windthermal.h"

#include <stdio.h>
#include <stdlib.h>

#include "util.h"
#include "../crrc_main.h"
#include "../global.h"
#include "../mod_misc/lib_conversions.h"
#include "../mod_misc/SimpleXMLTransfer.h"
#include "../mod_windfield/windfield.h"
#include "../mod_landscape/crrc_scenery.h"

static void CGUIWindThermalCallback(puObject *obj);
static void CGUIWindPresetCallback(puObject *obj);
static void CGUIThermalPresetCallback(puObject *obj);
static void CGUIWindNewPresetCallback(puObject *obj);
static void CGUIThermalNewPresetCallback(puObject *obj);

#define BUTTON_BOX_HEIGHT   (2*DLG_DEF_SPACE+DLG_DEF_BUTTON_HEIGHT)
#define SLIDER_W            330
#define SLIDER_H            DLG_DEF_BUTTON_HEIGHT
#define COMBO_W             330
#define COMBO_H             DLG_DEF_BUTTON_HEIGHT
#define LABEL_W             250
#define NUM_W               100

CGUIWindThermalDialog::CGUIWindThermalDialog()
  : CRRCDialog()
{
  SimpleXMLTransfer* thermaldata = cfg->getCurLocCfgPtr(cfgfile)->getChild("thermal");
  if (thermaldata->indexOfChild("v3") < 0)
  {
    thermalv3data = 0;
    std::cout << "Current thermal model is v1\n";
  }
  else
  {
    thermalv3data = thermaldata->getChild("v3");
    std::cout << "Current thermal model is v3\n";
  }
    
  // Load presets
  presetGrpWind    = cfgfile->getChild("presets.wind", true);
  presetsWind      = T_GUI_Util::loadnames(presetGrpWind, nPresetsWind);
  
  presetGrpThermal = cfgfile->getChild("presets.thermal", true);
  presetsThermal   = T_GUI_Util::loadnames(presetGrpThermal, nPresetsThermal);

  // Create widgets
  //
  comboPresetsWind = new puaComboBox(LABEL_W + DLG_DEF_SPACE, BUTTON_BOX_HEIGHT + 13* DLG_DEF_SPACE + 12* SLIDER_H + 1*COMBO_H,
                                     LABEL_W + COMBO_W,       BUTTON_BOX_HEIGHT + 13* DLG_DEF_SPACE + 12* SLIDER_H + 2*COMBO_H,
                                     NULL, false);
  comboPresetsWind->setChildColourScheme(PUCLASS_POPUPMENU, dlgCol1[0], dlgCol1[1], dlgCol1[2]);
  comboPresetsWind->newList(presetsWind);
  comboPresetsWind->setLabelPlace(PUPLACE_CENTERED_LEFT);
  comboPresetsWind->setLabel(_("Load Preset"));
  comboPresetsWind->setCurrentItem(0);
  comboPresetsWind->setCallback(CGUIWindPresetCallback);
  comboPresetsWind->reveal();
  comboPresetsWind->activate();
  comboPresetsWind->setUserData(this);

  slider_windVelocity = new crrcSlider(LABEL_W + DLG_DEF_SPACE,            BUTTON_BOX_HEIGHT + 12* DLG_DEF_SPACE + 11* SLIDER_H + 1*COMBO_H,
                                       LABEL_W + DLG_DEF_SPACE + SLIDER_W, BUTTON_BOX_HEIGHT + 12* DLG_DEF_SPACE + 12* SLIDER_H + 1*COMBO_H,
                                       NUM_W);
  slider_windVelocity->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_windVelocity->setLabel(_("Velocity [ft/s]"));
  slider_windVelocity->setSliderFraction(0.05);
  slider_windVelocity->setMinValue(0);
  slider_windVelocity->setMaxValue(80);
  slider_windVelocity->setStepSize(1);
  slider_windVelocity->setValue(cfg->wind->getVelocity());

  slider_windDir = new crrcSlider(LABEL_W + DLG_DEF_SPACE,            BUTTON_BOX_HEIGHT + 11* DLG_DEF_SPACE + 10* SLIDER_H + 1*COMBO_H,
                                  LABEL_W + DLG_DEF_SPACE + SLIDER_W, BUTTON_BOX_HEIGHT + 11* DLG_DEF_SPACE + 11* SLIDER_H + 1*COMBO_H,
                                  NUM_W);
  slider_windDir->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_windDir->setLabel(_("Direction"));
  slider_windDir->setSliderFraction(0.05);
  slider_windDir->setMinValue(0);
  slider_windDir->setMaxValue(359);
  slider_windDir->setStepSize(1);
  slider_windDir->setValue(cfg->wind->getDirection());
  //if direction imposed on scenery file description invalidate wind slider :
  if( Global::scenery->getImposeWindDirection() ) slider_windDir->greyOut();

  // Use turbulence model check box
  slider_windTurbulence = new crrcSlider(LABEL_W + DLG_DEF_SPACE,            BUTTON_BOX_HEIGHT + 10* DLG_DEF_SPACE +  9* SLIDER_H + 1*COMBO_H,
                                         LABEL_W + DLG_DEF_SPACE + SLIDER_W, BUTTON_BOX_HEIGHT + 10* DLG_DEF_SPACE + 10* SLIDER_H + 1*COMBO_H,
                                         NUM_W);
  slider_windTurbulence->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_windTurbulence->setLabel(_("Turbulence intensity [rel]"));
  slider_windTurbulence->setSliderFraction(0.05);
  slider_windTurbulence->setMinValue(0);
  slider_windTurbulence->setMaxValue(2);
  slider_windTurbulence->setStepSize(0.2);
  slider_windTurbulence->setValue(cfg->wind->getTurbulence());
  
  inputNewWind = new puInput(DLG_DEF_SPACE,           BUTTON_BOX_HEIGHT + 9* DLG_DEF_SPACE + 8* SLIDER_H + 1*COMBO_H,
                             LABEL_W + DLG_DEF_SPACE, BUTTON_BOX_HEIGHT + 9* DLG_DEF_SPACE + 9* SLIDER_H + 1*COMBO_H);
  inputNewWind->setValue(_("Name of new preset"));
  
  puOneShot* buttonTmp = new puOneShot(LABEL_W + DLG_DEF_SPACE, BUTTON_BOX_HEIGHT + 9* DLG_DEF_SPACE + 8* SLIDER_H + 1*COMBO_H,
                                       LABEL_W + COMBO_W,       BUTTON_BOX_HEIGHT + 9* DLG_DEF_SPACE + 9* SLIDER_H + 1*COMBO_H);
  buttonTmp->setLegend(_("Save as new preset"));
  buttonTmp->setCallback(CGUIWindNewPresetCallback);
  buttonTmp->setUserData(this);          
  
  comboPresetsThermal = new puaComboBox(LABEL_W + DLG_DEF_SPACE, BUTTON_BOX_HEIGHT + 8* DLG_DEF_SPACE + 8* SLIDER_H + 0*COMBO_H,
                                       LABEL_W + COMBO_W,       BUTTON_BOX_HEIGHT + 8* DLG_DEF_SPACE + 8* SLIDER_H + 1*COMBO_H,
                                       NULL, false);
  comboPresetsThermal->setChildColourScheme(PUCLASS_POPUPMENU, dlgCol1[0], dlgCol1[1], dlgCol1[2]);
  comboPresetsThermal->newList(presetsThermal);
  comboPresetsThermal->setLabelPlace(PUPLACE_CENTERED_LEFT);
  comboPresetsThermal->setLabel(_("Load Preset"));
  comboPresetsThermal->setCurrentItem(0);
  comboPresetsThermal->setCallback(CGUIThermalPresetCallback);
  comboPresetsThermal->reveal();
  comboPresetsThermal->activate();
  comboPresetsThermal->setUserData(this);

  slider_thermalStrengthMean = new crrcSlider(LABEL_W + DLG_DEF_SPACE,            BUTTON_BOX_HEIGHT + 7* DLG_DEF_SPACE + 7* SLIDER_H,
                                              LABEL_W + DLG_DEF_SPACE + SLIDER_W, BUTTON_BOX_HEIGHT + 7* DLG_DEF_SPACE + 8* SLIDER_H,
                                              NUM_W);
  slider_thermalStrengthMean->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_thermalStrengthMean->setLabel(_("Strength mean [ft/s]"));
  slider_thermalStrengthMean->setSliderFraction(0.05);
  slider_thermalStrengthMean->setMinValue(0);
  slider_thermalStrengthMean->setMaxValue(30);
  slider_thermalStrengthMean->setStepSize(0.5);
  slider_thermalStrengthMean->setValue((float)(thermaldata->getDouble("strength_mean")));

  slider_thermalStrengthSigma = new crrcSlider(LABEL_W + DLG_DEF_SPACE,            BUTTON_BOX_HEIGHT + 6* DLG_DEF_SPACE + 6* SLIDER_H,
                                               LABEL_W + DLG_DEF_SPACE + SLIDER_W, BUTTON_BOX_HEIGHT + 6* DLG_DEF_SPACE + 7* SLIDER_H,
                                               NUM_W);
  slider_thermalStrengthSigma->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_thermalStrengthSigma->setLabel(_("Strength sigma"));
  slider_thermalStrengthSigma->setSliderFraction(0.05);
  slider_thermalStrengthSigma->setMinValue(0);
  slider_thermalStrengthSigma->setMaxValue(15);
  slider_thermalStrengthSigma->setStepSize(0.2);
  slider_thermalStrengthSigma->setValue((float)(thermaldata->getDouble("strength_sigma")));

  slider_thermalRadiusMean = new crrcSlider(LABEL_W + DLG_DEF_SPACE,            BUTTON_BOX_HEIGHT + 5* DLG_DEF_SPACE + 5* SLIDER_H,
                                            LABEL_W + DLG_DEF_SPACE + SLIDER_W, BUTTON_BOX_HEIGHT + 5* DLG_DEF_SPACE + 6* SLIDER_H,
                                            NUM_W);
  slider_thermalRadiusMean->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_thermalRadiusMean->setLabel(_("Radius mean [ft]"));
  slider_thermalRadiusMean->setSliderFraction(0.05);
  slider_thermalRadiusMean->setMinValue(0);
  slider_thermalRadiusMean->setMaxValue(200);
  slider_thermalRadiusMean->setStepSize(1);
  slider_thermalRadiusMean->setValue((float)(thermaldata->getDouble("radius_mean")));

  slider_thermalRadiusSigma = new crrcSlider(LABEL_W + DLG_DEF_SPACE,            BUTTON_BOX_HEIGHT + 4* DLG_DEF_SPACE + 4* SLIDER_H,
                                             LABEL_W + DLG_DEF_SPACE + SLIDER_W, BUTTON_BOX_HEIGHT + 4* DLG_DEF_SPACE + 5* SLIDER_H,
                                             NUM_W);
  slider_thermalRadiusSigma->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_thermalRadiusSigma->setLabel(_("Radius sigma"));
  slider_thermalRadiusSigma->setSliderFraction(0.05);
  slider_thermalRadiusSigma->setMinValue(0);
  slider_thermalRadiusSigma->setMaxValue(150);
  slider_thermalRadiusSigma->setStepSize(0.5);
  slider_thermalRadiusSigma->setValue((float)(thermaldata->getDouble("radius_sigma")));

  slider_thermalLifetimeMean = new crrcSlider(LABEL_W + DLG_DEF_SPACE,            BUTTON_BOX_HEIGHT + 3* DLG_DEF_SPACE + 3* SLIDER_H,
                                              LABEL_W + DLG_DEF_SPACE + SLIDER_W, BUTTON_BOX_HEIGHT + 3* DLG_DEF_SPACE + 4* SLIDER_H,
                                              NUM_W);
  slider_thermalLifetimeMean->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_thermalLifetimeMean->setLabel(_("Lifetime mean [seconds]"));
  slider_thermalLifetimeMean->setSliderFraction(0.05);
  slider_thermalLifetimeMean->setMinValue(0);
  slider_thermalLifetimeMean->setMaxValue(300);
  slider_thermalLifetimeMean->setStepSize(1);
  slider_thermalLifetimeMean->setValue((float)(thermaldata->getDouble("lifetime_mean")));

  slider_thermalLifetimeSigma = new crrcSlider(LABEL_W + DLG_DEF_SPACE,            BUTTON_BOX_HEIGHT + 2* DLG_DEF_SPACE + 2* SLIDER_H,
                                               LABEL_W + DLG_DEF_SPACE + SLIDER_W, BUTTON_BOX_HEIGHT + 2* DLG_DEF_SPACE + 3* SLIDER_H,
                                               NUM_W);
  slider_thermalLifetimeSigma->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_thermalLifetimeSigma->setLabel(_("Lifetime sigma"));
  slider_thermalLifetimeSigma->setSliderFraction(0.05);
  slider_thermalLifetimeSigma->setMinValue(0);
  slider_thermalLifetimeSigma->setMaxValue(300);
  slider_thermalLifetimeSigma->setStepSize(1);
  slider_thermalLifetimeSigma->setValue((float)(thermaldata->getDouble("lifetime_sigma")));

  slider_thermalDensity = new crrcSlider(LABEL_W + DLG_DEF_SPACE,            BUTTON_BOX_HEIGHT + 1* DLG_DEF_SPACE + 1* SLIDER_H,
                                         LABEL_W + DLG_DEF_SPACE + SLIDER_W, BUTTON_BOX_HEIGHT + 1* DLG_DEF_SPACE + 2* SLIDER_H,
                                         NUM_W);
  slider_thermalDensity->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_thermalDensity->setLabel(_("Density [1/sq foot]"));
  slider_thermalDensity->setSliderFraction(0.05);
  slider_thermalDensity->setMinValue(0);
  slider_thermalDensity->setMaxValue(getMaxThermalDensity());
  slider_thermalDensity->setStepSize(2.0E-7);
  slider_thermalDensity->setValue((float)(thermaldata->getDouble("density")));

  inputNewThermal = new puInput(DLG_DEF_SPACE,           BUTTON_BOX_HEIGHT + 0* DLG_DEF_SPACE + 0* SLIDER_H + 0*COMBO_H,
                                LABEL_W + DLG_DEF_SPACE, BUTTON_BOX_HEIGHT + 0* DLG_DEF_SPACE + 1* SLIDER_H + 0*COMBO_H);
  inputNewThermal->setValue(_("Name of new preset"));
  
  buttonTmp = new puOneShot(LABEL_W + DLG_DEF_SPACE, BUTTON_BOX_HEIGHT + 0* DLG_DEF_SPACE + 0* SLIDER_H + 0*COMBO_H,
                            LABEL_W + COMBO_W,       BUTTON_BOX_HEIGHT + 0* DLG_DEF_SPACE + 1* SLIDER_H + 0*COMBO_H);
  buttonTmp->setLegend(_("Save as new preset"));
  buttonTmp->setCallback(CGUIThermalNewPresetCallback);
  buttonTmp->setUserData(this);          
      
  // finalize the dialog
  close();
  setSize(SLIDER_W + LABEL_W + 2*DLG_DEF_SPACE,
          12*SLIDER_H + 14*DLG_DEF_SPACE + 2*COMBO_H + BUTTON_BOX_HEIGHT);
  setCallback(CGUIWindThermalCallback);
  centerOnScreen();
  reveal();
}

/**
 * Destroy the dialog.
 */

CGUIWindThermalDialog::~CGUIWindThermalDialog()
{
  T_GUI_Util::freenames(presetsWind,    nPresetsWind);
  T_GUI_Util::freenames(presetsThermal, nPresetsThermal);
}

/** \brief callback to load a preset.
 *
 */
void CGUIWindPresetCallback(puObject *obj)
{
  CGUIWindThermalDialog* dlg   = (CGUIWindThermalDialog*)obj->getUserData();
  int                    nItem = dlg->comboPresetsWind->getCurrentItem();

  if (nItem == 0)
  {
    dlg->slider_windDir->setValue(270);
    dlg->slider_windVelocity->setValue(5);
    dlg->slider_windTurbulence->setValue(1);
  }
  else
  {
    SimpleXMLTransfer* preset = dlg->presetGrpWind->getChildAt(nItem-1);
    dlg->slider_windDir->setValue((float)preset->getDouble("direction",270.));
    dlg->slider_windVelocity->setValue((float)preset->getDouble("velocity",5.));
    dlg->slider_windTurbulence->setValue((float)preset->getDouble("turbulence",1.));
  }
}

/** \brief callback to save a preset.
 *
 */
void CGUIWindNewPresetCallback(puObject *obj)
{
  CGUIWindThermalDialog* dlg   = (CGUIWindThermalDialog*)obj->getUserData();
  
  SimpleXMLTransfer* wind      = new SimpleXMLTransfer();
  
  wind->setName("wind");
  wind->setAttribute("name_en",    dlg->inputNewWind->getStringValue());
  wind->setAttribute("velocity",   dlg->slider_windVelocity->getStringValue());
  wind->setAttribute("direction",  dlg->slider_windDir->getStringValue());
  wind->setAttribute("turbulence", dlg->slider_windTurbulence->getStringValue());
  
  cfgfile->getChild("presets.wind", true)->addChild(wind);
  
  // save file
  options_saveToFile();
  
  // exit dialog (otherwise we would have to regenerate presetsWind)
  dlg->setValue(CRRC_DIALOG_OK);
  CGUIWindThermalCallback(dlg);
}

/** \brief callback to save a preset.
 *
 */
void CGUIThermalNewPresetCallback(puObject *obj)
{
  CGUIWindThermalDialog* dlg   = (CGUIWindThermalDialog*)obj->getUserData();
  
  SimpleXMLTransfer* thermal   = new SimpleXMLTransfer();
  
  thermal->setName("thermal");
  thermal->setAttribute("name_en",         dlg->inputNewThermal->getStringValue());
  thermal->setAttribute("strength_mean",   dlg->slider_thermalStrengthMean->getStringValue());
  thermal->setAttribute("strength_sigma",  dlg->slider_thermalStrengthSigma->getStringValue());
  thermal->setAttribute("radius_mean",     dlg->slider_thermalRadiusMean->getStringValue());
  thermal->setAttribute("radius_sigma",    dlg->slider_thermalRadiusSigma->getStringValue());
  thermal->setAttribute("lifetime_mean",   dlg->slider_thermalLifetimeMean->getStringValue());
  thermal->setAttribute("lifetime_sigma",  dlg->slider_thermalLifetimeSigma->getStringValue());
  thermal->setAttribute("density",         dlg->slider_thermalDensity->getStringValue());
  if (dlg->thermalv3data != 0)
  {
    std::cout << "New preset is thermal model v3\n";
    thermal->addChild(new SimpleXMLTransfer(dlg->thermalv3data));
  }
  else
    std::cout << "New preset is thermal model v1\n";
  
  cfgfile->getChild("presets.thermal", true)->addChild(thermal);

  // show new preset (debug only)
  cfgfile->getChild("presets.thermal")->print(std::cout, 2);
  
  // save file
  options_saveToFile();
  
  // exit dialog (otherwise we would have to regenerate presetsThermal)
  dlg->setValue(CRRC_DIALOG_OK);
  CGUIWindThermalCallback(dlg);
}

/** \brief callback to load a preset.
 *
 */
void CGUIThermalPresetCallback(puObject *obj)
{
  CGUIWindThermalDialog* dlg   = (CGUIWindThermalDialog*)obj->getUserData();
  int                    nItem = dlg->comboPresetsThermal->getCurrentItem();

  if (nItem == 0)
  {
    dlg->slider_thermalStrengthMean->setValue(5);
    dlg->slider_thermalStrengthSigma->setValue(1);
    dlg->slider_thermalRadiusMean->setValue(50);
    dlg->slider_thermalRadiusSigma->setValue(10);
    dlg->slider_thermalLifetimeMean->setValue(240);
    dlg->slider_thermalLifetimeSigma->setValue(60);
    dlg->slider_thermalDensity->setValue((float)0.5E-5);

    // v3-Data is removed, this preset is v1:
    dlg->thermalv3data = 0;
  }
  else
  {
    SimpleXMLTransfer* preset = dlg->presetGrpThermal->getChildAt(nItem-1);
    dlg->slider_thermalStrengthMean->setValue((float)preset->getDouble("strength_mean"));
    dlg->slider_thermalStrengthSigma->setValue((float)preset->getDouble("strength_sigma"));
    dlg->slider_thermalRadiusMean->setValue((float)preset->getDouble("radius_mean"));
    dlg->slider_thermalRadiusSigma->setValue((float)preset->getDouble("radius_sigma"));
    dlg->slider_thermalLifetimeMean->setValue((float)preset->getDouble("lifetime_mean"));
    dlg->slider_thermalLifetimeSigma->setValue((float)preset->getDouble("lifetime_sigma"));
    dlg->slider_thermalDensity->setValue((float)preset->getDouble("density"));

    if (preset->indexOfChild("v3") < 0)
    {
      std::cout << "Chosen preset is thermal model v1\n";
      dlg->thermalv3data = 0;
    }
    else
    {
      std::cout << "Chosen preset is thermal model v3\n";
      dlg->thermalv3data = preset->getChild("v3");
    }
  }
}

/** \brief The dialog's callback.
 *
 */
void CGUIWindThermalCallback(puObject *obj)
{
  if (obj->getIntegerValue() == CRRC_DIALOG_OK)
  {
    // Dialog left by clicking OK
    CGUIWindThermalDialog* dlg   = (CGUIWindThermalDialog*)obj;
    
    cfg->wind->setDirection(dlg->slider_windDir->getFloatValue(), cfg);
    cfg->wind->setVelocity(dlg->slider_windVelocity->getFloatValue());
    cfg->wind->setTurbulence(dlg->slider_windTurbulence->getFloatValue());
        
    SimpleXMLTransfer* thermaldata = cfg->getCurLocCfgPtr(cfgfile)->getChild("thermal");
    thermaldata->setAttributeOverwrite("strength_mean",  dlg->slider_thermalStrengthMean->getStringValue());
    thermaldata->setAttributeOverwrite("strength_sigma", dlg->slider_thermalStrengthSigma->getStringValue());
    thermaldata->setAttributeOverwrite("radius_mean",    dlg->slider_thermalRadiusMean->getStringValue());
    thermaldata->setAttributeOverwrite("radius_sigma",   dlg->slider_thermalRadiusSigma->getStringValue());
    thermaldata->setAttributeOverwrite("lifetime_mean",  dlg->slider_thermalLifetimeMean->getStringValue());
    thermaldata->setAttributeOverwrite("lifetime_sigma", dlg->slider_thermalLifetimeSigma->getStringValue());
    thermaldata->setAttributeOverwrite("density",        dlg->slider_thermalDensity->getStringValue());

    // handle v3 data
    {
      int n_v3idx = thermaldata->indexOfChild("v3");
            
      if (dlg->thermalv3data == 0)
      {
        if (n_v3idx >= 0)  // v3 has to be deleted after loading a preset without v3:
        {
          std::cout << "Chosen preset is thermal model v1, so v3 data is removed.\n";        
          SimpleXMLTransfer* tmp = thermaldata->getChildAt(n_v3idx);
          thermaldata->removeChildAt(n_v3idx);
          delete tmp;
        }
      }
      else
      {
        // there is no v3 data yet?
        if (n_v3idx < 0)
        {
          std::cout << "Load data for thermal model v3.\n";
          thermaldata->addChild(new SimpleXMLTransfer(dlg->thermalv3data));
        }
        else
        {
          if (dlg->thermalv3data != thermaldata->getChildAt(n_v3idx))
          {
            std::cout << "Chosen preset is thermal model v3 and we currently have v3 which differs from chosen preset. Remove current data.\n";
            SimpleXMLTransfer* tmp = thermaldata->getChildAt(n_v3idx);
            thermaldata->removeChildAt(n_v3idx);
            delete tmp;
            
            std::cout << "Load data for thermal model v3.\n";
            thermaldata->addChild(new SimpleXMLTransfer(dlg->thermalv3data));
          }
        }   
      }
    }
    
    // Thermals have to be recreated:
    clear_wind_field();
    cfg->thermal->read(cfgfile, cfg);
    Init_mod_windfield();
  }
  
  std::cout.flush();
  
  puDeleteObject(obj);
}
