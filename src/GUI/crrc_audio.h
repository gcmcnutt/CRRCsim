/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2008 Jens Wilhelm Wulf (original author)
 * Copyright (C) 2008, 2009 Jan Reucker
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
  

/// \file crrc_audio.h - Sound settings dialog

#ifndef CRRC_AUDIO_H
#define CRRC_AUDIO_H

#include <plib/pu.h>
#include <plib/puAux.h>

#include "crrc_dialog.h"
#include "../mod_misc/SimpleXMLTransfer.h"

class CGUIAudioDialog;

/** \brief The Audio options dialog.
 *
 */
class CGUIAudioDialog : public CRRCDialog
{
  public:
    CGUIAudioDialog();
    ~CGUIAudioDialog();
   
    puaComboBox*        comboSampleRate;
    static const char*  samplerates[];
   
    puaComboBox*        comboEngine;
    static const char*  enginesound[];
   
    puSlider*           slider_variovol;
   
    puSlider*           slider_modelvol;
};

#endif // CRRC_AUDIO_H
