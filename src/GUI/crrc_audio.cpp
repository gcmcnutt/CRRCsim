/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2008 Jens Wilhelm Wulf (original author)
 * Copyright (C) 2005, 2006, 2008, 2009 Jan Reucker
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
  

/// \file crrc_audio.cpp
///       Implementation of class CGUIAudioDialog

#include "../i18n.h"
#include "crrc_audio.h"

#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "../global.h"
#include "util.h"
#include "../crrc_main.h"
#include "../mod_misc/lib_conversions.h"
#include "../mod_misc/SimpleXMLTransfer.h"
#include "../crrc_sound.h"

static void CGUIAudioCallback(puObject *obj);

#define BUTTON_BOX_HEIGHT   (2*DLG_DEF_SPACE+DLG_DEF_BUTTON_HEIGHT)
#define SLIDER_W            330
#define SLIDER_H            DLG_DEF_BUTTON_HEIGHT
#define COMBO_H             DLG_DEF_BUTTON_HEIGHT
#define COMBO_W             330
#define LABEL_W             200
#define NUM_W               70

const char* CGUIAudioDialog::samplerates[] = {"22050 Hz", "44100 Hz", "48000 Hz", NULL};
const char* CGUIAudioDialog::enginesound[] = {_("Off"), _("On"), _("Always on"), NULL};

CGUIAudioDialog::CGUIAudioDialog() 
            : CRRCDialog()
{
  // Create Widgets
  comboSampleRate = new puaComboBox(LABEL_W + DLG_DEF_SPACE,
                                    BUTTON_BOX_HEIGHT + 4 * DLG_DEF_SPACE + 2 * SLIDER_H + COMBO_H,
                                    LABEL_W + COMBO_W,
                                    BUTTON_BOX_HEIGHT + 4 * DLG_DEF_SPACE + 2 * SLIDER_H + 2 * COMBO_H,
                                    NULL, false);
  comboSampleRate->setChildColourScheme(PUCLASS_POPUPMENU, dlgCol1[0], dlgCol1[1], dlgCol1[2]);
  comboSampleRate->newList((char**) samplerates);
  comboSampleRate->setLabelPlace(PUPLACE_CENTERED_LEFT);
  comboSampleRate->setLabel(_("Sample Rate"));
  {
    int         nIdx = 0;
    std::string cr   = cfgfile->getString("sound.samplerate");
    std::string tmp;
    
    for (int n=0; n<3; n++)
    {
      tmp = samplerates[n];
      if (tmp.find(cr) == 0)
        nIdx = n;
    }
    comboSampleRate->setCurrentItem(nIdx);
  }
  comboSampleRate->reveal();
  
  comboEngine = new puaComboBox(LABEL_W + DLG_DEF_SPACE, 
                                BUTTON_BOX_HEIGHT + 3 * DLG_DEF_SPACE + 2 * SLIDER_H,
                                LABEL_W + COMBO_W,
                                BUTTON_BOX_HEIGHT + 3 * DLG_DEF_SPACE + 2 * SLIDER_H + COMBO_H,
                                NULL, false);
  comboEngine->setChildColourScheme(PUCLASS_POPUPMENU, dlgCol1[0], dlgCol1[1], dlgCol1[2]);
  comboEngine->newList((char**) enginesound);
  comboEngine->setLabelPlace(PUPLACE_CENTERED_LEFT);
  comboEngine->setLabel(_("Engine sound"));
  comboEngine->setCurrentItem(cfgfile->getInt("sound.throttle.mode", 1));
  comboEngine->hide();
  
  slider_variovol = new puSlider(LABEL_W + DLG_DEF_SPACE,
                                 BUTTON_BOX_HEIGHT + 2 * DLG_DEF_SPACE + 1 * SLIDER_H,
                                 SLIDER_W,
                                 FALSE, 
                                 SLIDER_H);
  slider_variovol->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_variovol->setLabel(_("Variometer volume"));
  slider_variovol->setSliderFraction(0.05);
  slider_variovol->setMinValue(0);
  slider_variovol->setMaxValue(16);
  slider_variovol->setStepSize(1);
  slider_variovol->setValue(cfgfile->getInt("sound.variometer.vol"));
  
  slider_modelvol = new puSlider(LABEL_W + DLG_DEF_SPACE,
                                 BUTTON_BOX_HEIGHT + 1 * DLG_DEF_SPACE + 0* SLIDER_H,
                                 SLIDER_W,
                                 FALSE, 
                                 SLIDER_H);
  slider_modelvol->setLabelPlace(PUPLACE_CENTERED_LEFT);
  slider_modelvol->setLabel(_("Model volume"));
  slider_modelvol->setSliderFraction(0.05);
  slider_modelvol->setMinValue(0);
  slider_modelvol->setMaxValue(1.0);
  slider_modelvol->setStepSize(0.05);
  slider_modelvol->setValue((float)Global::soundserver->getModelVolume() / (float)SDL_MIX_MAXVOLUME);
  
  // finalize the dialog
  close();
  setSize(SLIDER_W + LABEL_W + 2*DLG_DEF_SPACE, 
          2*SLIDER_H + 5*DLG_DEF_SPACE + 2 * COMBO_H + BUTTON_BOX_HEIGHT);
  setCallback(CGUIAudioCallback);
  centerOnScreen();
  reveal();
}


/**
 * Destroy the dialog.
 */

CGUIAudioDialog::~CGUIAudioDialog()
{
}

/** \brief The dialog's callback.
 *
 */
void CGUIAudioCallback(puObject *obj)
{
  if (obj->getIntegerValue() == CRRC_DIALOG_OK)
  {
    CGUIAudioDialog *dlg = (CGUIAudioDialog*)obj;
    // Dialog left by clicking OK
    cfgfile->setAttributeOverwrite("sound.variometer.vol",
                                   dlg->slider_variovol->getStringValue());
    //~ cfgfile->setAttributeOverwrite("sound.throttle.mode",
                                   //~ dlg->comboEngine->getCurrentItem());
    {
      std::string SR = CGUIAudioDialog::samplerates[dlg->comboSampleRate->getCurrentItem()];
      
      int nSR  = atoi(SR.substr(0, SR.find(' ')).c_str());
      cfgfile->setAttributeOverwrite("sound.samplerate", nSR);
      
      int varvol = dlg->slider_variovol->getIntegerValue() << 3;
      printf("Setting vario volume to %d\n", varvol);
      Global::soundserver->setChannelVolume(vario_sound_channel, varvol);
      
      unsigned char ucModelVol = dlg->slider_modelvol->getFloatValue() * SDL_MIX_MAXVOLUME;
      std::cout << "Setting model volume to " << (int)ucModelVol << std::endl;
      Global::soundserver->setModelVolume(ucModelVol);
      Global::soundserver->putBackIntoConfig(cfgfile);
    }    
  }
 
  puDeleteObject(obj);
}
