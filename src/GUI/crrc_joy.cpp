/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2008 Jens Wilhelm Wulf (original author)
 * Copyright (C) 2008, 2009 Jan Reucker
 * Copyright (C) 2008 Olivier Bordes
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
  

// implementation of class CGUIJoystickDialog

// always include configuration header first
#include "../i18n.h"
#include <crrc_config.h>

#include "../global.h"
#include "crrc_joy.h"
#include "crrc_msgbox.h"
#include "../mouse_kbd.h"
#include "../crrc_main.h"
#include "../mod_misc/lib_conversions.h"


CGUIJoystickDialog* joystickDlg  = (CGUIJoystickDialog*)0;
int                 nJoystickDlg = 0;

static void CGUIJoystickCallback(puObject *obj);
static void CGUIJoystickButtonCallback(puObject *obj);

#define BUTTON_BOX_HEIGHT   (2*DLG_DEF_SPACE+DLG_DEF_BUTTON_HEIGHT)
#define BUTTON_W            200
#define BUTTON_H            DLG_DEF_BUTTON_HEIGHT
#define COMBO_W             200
#define COMBO_H             DLG_DEF_BUTTON_HEIGHT
#define LABEL_W             170
#define NUM_W               70

CGUIJoystickDialog::CGUIJoystickDialog() 
            : CRRCDialog(0, 0, CRRC_DIALOG_OK)
{
  // Create Widgets
  puText* info = new puText(DLG_DEF_SPACE, BUTTON_BOX_HEIGHT + 3* DLG_DEF_SPACE + 2* COMBO_H);
  info->setLabelPlace(PUPLACE_LOWER_RIGHT);
  info->setLabel(_("Hit a button, then choose a function and bind..."));


  comboButton = new puaComboBox(LABEL_W + DLG_DEF_SPACE, BUTTON_BOX_HEIGHT + 1* DLG_DEF_SPACE + 1* COMBO_H,
                               LABEL_W + COMBO_W,       BUTTON_BOX_HEIGHT + 1* DLG_DEF_SPACE + 2* COMBO_H,
                               NULL, false);
  comboButton->setChildColourScheme(PUCLASS_POPUPMENU, dlgCol1[0], dlgCol1[1], dlgCol1[2]);
  comboButton->newList(Global::inputDev->ActionButtonStringsGUI);
  comboButton->setLabelPlace(PUPLACE_LOWER_LEFT);
  comboButton->setLabel(_("Press some button!"));
  comboButton->setCurrentItem(0);

  buttonBindButton = new puOneShot(LABEL_W + COMBO_W + DLG_DEF_SPACE, BUTTON_BOX_HEIGHT + 1* DLG_DEF_SPACE + 1* COMBO_H,
                                   LABEL_W + COMBO_W + DLG_DEF_SPACE + BUTTON_W, BUTTON_BOX_HEIGHT + 1* DLG_DEF_SPACE + 1* COMBO_H + BUTTON_H);
  buttonBindButton->setLegend(_("Bind Button"));
  buttonBindButton->setCallback(CGUIJoystickButtonCallback);
  buttonBindButton->setUserData(this);

  nButton = -1;

  // finalize the dialog
  close();
  setSize(BUTTON_W + COMBO_W + LABEL_W + 2*DLG_DEF_SPACE,
          BUTTON_BOX_HEIGHT + 3* DLG_DEF_SPACE + 4* COMBO_H);
  setCallback(CGUIJoystickCallback);
  centerOnScreen();
  reveal();
}


/**
 * Destroy the dialog.
 */

CGUIJoystickDialog::~CGUIJoystickDialog()
{
}

/** \brief The dialog's callback.
 *
 */
void CGUIJoystickCallback(puObject *obj)
{
  nJoystickDlg = 0;
  
  if (obj->getIntegerValue() == CRRC_DIALOG_OK)
  {
    // Dialog left by clicking OK
  }
  
  puDeleteObject(obj);  
}

/** \brief callback to bind something
 *
 */
void CGUIJoystickButtonCallback(puObject *obj)
{
  CGUIJoystickDialog* dlg = (CGUIJoystickDialog*)obj->getUserData();
  
  if (dlg->nButton >= 0)
  {
    Global::inputDev->joystick_bind_b[dlg->nButton] = dlg->comboButton->getCurrentItem();
  }
  else
  {
    new CGUIMsgBox(_("Please press button first!"));
  }
}

void CGUIJoystickDialog::joystickDlgButton(SDL_JoyButtonEvent *event)
{
  int         nButton = event->button;
  
  if (event->state != SDL_PRESSED || event->button > MAXJOYBUTTON)
    return;
  
  joystickDlg->nButton = nButton;
  
  // Set label
  textButton = _("Button ") + itoStr(nButton, ' ', 2);
  joystickDlg->comboButton->setLabel(textButton.c_str());
  
  // Update combo box
  joystickDlg->comboButton->setCurrentItem(Global::inputDev->joystick_bind_b[nButton]);
}

