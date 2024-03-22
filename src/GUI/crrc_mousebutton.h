/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2008 Jens Wilhelm Wulf (original author)
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
  


#ifndef CRRC_MOUSEBUTTON_H
#define CRRC_MOUSEBUTTON_H

#include <plib/pu.h>
#include <plib/puAux.h>

#include "crrc_dialog.h"

class CGUIMouseButtonDialog;

/** \brief The MouseButton options dialog.
 *
 */
class CGUIMouseButtonDialog : public CRRCDialog
{
  public:
    CGUIMouseButtonDialog();
    ~CGUIMouseButtonDialog();
   
    puaComboBox* combo_l;
    puaComboBox* combo_m;
    puaComboBox* combo_r;
    puaComboBox* combo_up;
    puaComboBox* combo_down;
};

#endif // CRRC_MOUSEBUTTON_H
