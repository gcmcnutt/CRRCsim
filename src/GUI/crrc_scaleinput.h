/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2008 Jens Wilhelm Wulf (original author)
 * Copyright (C) 2005 Jan Reucker
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
 *  \file crrc_scaleinput
 *  Declaration of class CGUIMixerDialog
 */


#ifndef CRRC_SCALEINPUT_H
#define CRRC_SCALEINPUT_H

#include <plib/pu.h>
#include <plib/puAux.h>

#include "crrc_dialog.h"
#include "crrc_slider.h"
#include "../mod_misc/SimpleXMLTransfer.h"
#include "../mod_inputdev/inputdev.h"

class CGUIMixerDialog;

/** \brief The software mixer options dialog.
 *
 */
class CGUIMixerDialog : public CRRCDialog
{
  public:
    enum { NrOfAxes = 6 };
    enum { NrOfMixers = T_TX_Mixer::NUM_MIXERS };
  
    CGUIMixerDialog(T_TX_Interface* itxi);
    ~CGUIMixerDialog();

    T_TX_Interface* txi;
    
    SimpleXMLTransfer* presetGrp;
    int                nPresets;

    puaComboBox*    comboPresets;
    puInput*        inputNewName;
    char**          presets;

    crrcSlider*     slider_trim[NrOfAxes];
    crrcSlider*     slider_nrate[NrOfAxes];
    crrcSlider*     slider_srate[NrOfAxes];
    crrcSlider*     slider_exp[NrOfAxes];

    puText*         labels[4];

    puButton*       enable_button;
    puButton*       dr_enable_button;

    puaComboBox*    combo_mix_src[NrOfMixers];
    puaComboBox*    combo_mix_dst[NrOfMixers];
    
    crrcSlider*     slider_mix_val[NrOfMixers];
   
    puText*         mixer_labels[4];

    puButton*       mix_enable_button[NrOfMixers];
    
    puGroup*        all_widgets;
};

#endif // CRRC_SCALEINPUT_H
