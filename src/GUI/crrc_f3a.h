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
  

// crrc_f3a.h - F3A game options dialog

#ifndef CRRC_GUIF3A_H
#define CRRC_GUIF3A_H

#include <plib/pu.h>
#include <plib/puAux.h>

#include "crrc_dialog.h"
#include "crrc_slider.h"
#include "../mod_misc/SimpleXMLTransfer.h"

class CGUIF3ADialog;

/** \brief The F3A options dialog.
 *
 */
class CGUIF3ADialog : public CRRCDialog
{
  public:
    CGUIF3ADialog();
    ~CGUIF3ADialog();

    puaComboBox*  comboPresets;
    char**        presets;

    puButton*     f3a_enable;
    puButton*     f3a_draw_grid;
    puButton*     f3a_draw_trajectory;
    puButton*     f3a_draw_indicators;

    crrcSlider*   slider_persistence;
    crrcSlider*   slider_tolerance;
    crrcSlider*   slider_grid_size;
    crrcSlider*   slider_flight_height;
    crrcSlider*   slider_flight_distance;
    crrcSlider*   slider_orientation;
    crrcSlider*   slider_position_n;
    crrcSlider*   slider_position_e;

    puInput*      inputNewName;
    SimpleXMLTransfer* presetGrp;
    int                nPresets;
};

#endif // CRRC_GUIF3A_H
