/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005 Olivier Bordes (original author)
 * Copyright (C) 2005, 2008 Jan Reucker
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
  

// crrc_Launch.h - Airplane selection dialog

#ifndef CRRC_GUIF3F_H
#define CRRC_GUIF3F_H

#include <plib/pu.h>
#include <plib/puAux.h>

#include "crrc_dialog.h"
#include "crrc_slider.h"
#include "../mod_misc/SimpleXMLTransfer.h"

#define MAX_F3F_SOUND_DIRS  (50)

class CGUIF3FDialog;

/** \brief The F3F options dialog.
 *
 */
class CGUIF3FDialog : public CRRCDialog
{
  public:
    CGUIF3FDialog();
    ~CGUIF3FDialog();

    puaComboBox* comboPresets;
    char**      presets;

    puButton*   f3f_enable;
    puButton*   f3f_extend_bases;
    puButton*   start_on_left;

    crrcSlider*   slider_security_line;
    crrcSlider*   slider_bases_distance;
    crrcSlider*   slider_orientation;
    crrcSlider*   slider_position_n;
    crrcSlider*   slider_position_e;
    puaComboBox   *soundSelectBox;
    char         *sound_list[MAX_F3F_SOUND_DIRS + 1];

    puInput*      inputNewName;
    SimpleXMLTransfer* presetGrp;
    int                nPresets;
  

};

#endif // CRRC_GUIF3F_H
