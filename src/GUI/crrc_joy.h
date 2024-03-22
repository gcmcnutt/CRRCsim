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
  

// crrc_Joystick.h - Airplane selection dialog

#ifndef CRRC_JOYSTICK_H
#define CRRC_JOYSTICK_H

#include <plib/pu.h>
#include <plib/puAux.h>
#include <SDL.h>
#include <string>

#include "crrc_dialog.h"

class CGUIJoystickDialog;

/**
 * to be able to manipulate the dialog
 */
extern CGUIJoystickDialog* joystickDlg;

/**
 * shows whether CGUIJoystickDialog* joystickDlg is valid
 */
extern int                 nJoystickDlg;


/** \brief The Joystick options dialog.
 *
 */
class CGUIJoystickDialog : public CRRCDialog
{
  public:
    CGUIJoystickDialog();
    ~CGUIJoystickDialog();
   
   /**
    * indicate button press
    * Updates label at combo box to reflect the number of 
    * the button and the choice inside the combo box to 
    * the current binding of this button.
    */
   void joystickDlgButton(SDL_JoyButtonEvent *event);
   
   puaComboBox* comboButton;        ///< axis function ComboBox widget
   puOneShot*   buttonBindButton;   ///< "Bind" button widget
   
   std::string textButton;
   int         nButton;
};

#endif // CRRC_JOYSTICK_H
