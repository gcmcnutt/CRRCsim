/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2007, 2008 Jens Wilhelm Wulf (original author)
 * Copyright (C) 2005, 2006, 2007, 2008 Jan Reucker
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
  


#ifndef CRRC_CTRLGEN_H
#define CRRC_CTRLGEN_H

#include <plib/pu.h>
#include <plib/puAux.h>

#include "crrc_dialog.h"
#include "crrc_spin.h"

typedef struct
{
  int         id;
  std::string sDevName;
  int         iMaxIn;
  int         iMaxOut;
} T_AudioDevInfo;


/** \brief The CtrlGeneral options dialog.
 *
 */
class CGUICtrlGeneralDialog : public CRRCDialog
{
  public:
    CGUICtrlGeneralDialog();
    ~CGUICtrlGeneralDialog();
    
    void showHideButtons();
    void updateInputMethod();
   
    puText*       label_inputMethod;
    std::string   inputMethodString;
    std::string   sDeviceName;
    
    puaComboBox*  combo_zoom;
    puButton*     button_calibrate;
    puButton*     button_mixer;
    puButton*     button_joybuttons;
    puButton*     button_device;
};


#endif // CRRC_CTRLGEN_H
