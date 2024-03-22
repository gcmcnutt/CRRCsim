/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2007, 2008 Jens Wilhelm Wulf (original author)
 * Copyright (C) 2005, 2006, 2007, 2008 Jan Reucker
 * Copyright (C) 2006 Todd Templeton
 * Copyright (C) 2007, 2008 Chris Bayley
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
  
#include "../i18n.h"
#include "../global.h"
#include "../defines.h"
#include <crrc_config.h>
#include "crrc_gui_main.h"
#include "crrc_ctrlgen.h"
#include "crrc_ctrldev.h"
#include "crrc_msgbox.h"
#include "crrc_calibmap.h"
#include "crrc_mousebutton.h"
#include "crrc_scaleinput.h"
#include "crrc_joy.h"
#include "util.h"

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <plib/ul.h>

#include "../crrc_main.h"
#include "../mouse_kbd.h"
#include "../mod_misc/lib_conversions.h"


static void CGUICtrlGeneralCallback(puObject *obj);
static void CGUICalibrateButtonCallback(puObject *obj);
static void CGUIJoystickButtonCallback(puObject *obj);
static void CGUIMixerButtonCallback(puObject *obj);
static void CGUIDeviceButtonCallback(puObject *obj);

#define BUTTON_BOX_HEIGHT (2*DLG_DEF_SPACE+DLG_DEF_BUTTON_HEIGHT)
#define COMBO_W           (200)
#define COMBO_H           (DLG_DEF_BUTTON_HEIGHT)
#define LABEL_W           (140)

/*
 * Just for the record: This dialog changed several times. Here are the
 * different strategies how to handle a changed input method:
 *
 * First strategy:
 * Fall back to mouse input in case of failure.
 * Bad: Future config changes will be applied to the mouse configuration.
 *
 * Second strategy:
 * In case of failure, don't fall back to mouse input! Else all future
 * configuration will be applied to the mouse configuration, so there's
 * no chance to change an erroneous parameter if the first try to open
 * an interface fails.
 * Bad: if reconfiguration fails, Global::TXInterface will be set to NULL
 *      and other parts of the simulation will segfault when trying to
 *      access it.
 * 
 * Third strategy:
 * Even if the combo box has changed, don't reconfigure the interface
 * immediately. Instead, make sure that the interface is reconfigured
 * and working with the current interface type, device and speed
 * settings before opening any subdialog or leaving the dialog.
 * Bad: The buttons reflect the state of Global::TXInterface, not of the
 *      combo box. Some settings (e.g. if a mixer is assigned to the interface)
 *      can only be queried directly from the live object. Therefore buttons
 *      may remain active although something is not supported by the interface
 *      selected in the combo box. This is confusing to the user.
 *
 * Fourth (current) strategy:
 * The input method is selected in a sub-dialog. The main dialog only reflects
 * this setting in a text label. Therefore all buttons remain in their state
 * as long as no new device is selected.
 * Bad? Only time will tell.
 */
CGUICtrlGeneralDialog::CGUICtrlGeneralDialog()
            : CRRCDialog(0, 0, CRRC_DIALOG_OK)
{
  /// TODO: Limit string length of inputMethodString
  label_inputMethod = new puText( DLG_DEF_SPACE, 
                                  BUTTON_BOX_HEIGHT + 7*DLG_DEF_SPACE + 3*COMBO_H + 4*DLG_DEF_BUTTON_HEIGHT);
  updateInputMethod();

  button_calibrate = new puOneShot(LABEL_W + DLG_DEF_SPACE,
                                   BUTTON_BOX_HEIGHT + 4*DLG_DEF_SPACE + 2*COMBO_H + 2*DLG_DEF_BUTTON_HEIGHT,
                                   LABEL_W + COMBO_W,
                                   BUTTON_BOX_HEIGHT + 4*DLG_DEF_SPACE + 2*COMBO_H + 3*DLG_DEF_BUTTON_HEIGHT);
  button_calibrate->setLegend(_("Configure..."));
  button_calibrate->setUserData(this);
  button_calibrate->greyOut();
  button_calibrate->setCallback(CGUICalibrateButtonCallback);

  button_mixer = new puOneShot(LABEL_W + DLG_DEF_SPACE,
                               BUTTON_BOX_HEIGHT + 3*DLG_DEF_SPACE + 2*COMBO_H + 1*DLG_DEF_BUTTON_HEIGHT,
                               LABEL_W + COMBO_W,
                               BUTTON_BOX_HEIGHT + 3*DLG_DEF_SPACE + 2*COMBO_H + 2*DLG_DEF_BUTTON_HEIGHT);
  button_mixer->setLegend(_("Mixer..."));
  button_mixer->setUserData(this);
  button_mixer->greyOut();
  button_mixer->setCallback(CGUIMixerButtonCallback);

  button_joybuttons = new puOneShot(LABEL_W + DLG_DEF_SPACE,
                                   BUTTON_BOX_HEIGHT + 2*DLG_DEF_SPACE + 2*COMBO_H,
                                   LABEL_W + COMBO_W,
                                   BUTTON_BOX_HEIGHT + 2*DLG_DEF_SPACE + 2*COMBO_H + DLG_DEF_BUTTON_HEIGHT);
  button_joybuttons->setLegend(_("Buttons..."));
  button_joybuttons->setUserData(this);
  button_joybuttons->greyOut();
  button_joybuttons->setCallback(CGUIJoystickButtonCallback);
  
  button_device = new puOneShot(LABEL_W + DLG_DEF_SPACE,
                                BUTTON_BOX_HEIGHT + 6*DLG_DEF_SPACE + 3*COMBO_H + 3*DLG_DEF_BUTTON_HEIGHT,
                                LABEL_W + COMBO_W,
                                BUTTON_BOX_HEIGHT + 6*DLG_DEF_SPACE + 3*COMBO_H + 4*DLG_DEF_BUTTON_HEIGHT);
  button_device->setLegend(_("Input Method..."));
  button_device->setUserData(this);
  button_device->reveal();
  button_device->setCallback(CGUIDeviceButtonCallback);

  combo_zoom = new puaComboBox( LABEL_W + DLG_DEF_SPACE,
                               BUTTON_BOX_HEIGHT + 0*DLG_DEF_SPACE + 0*COMBO_H,
                               LABEL_W + COMBO_W,
                               BUTTON_BOX_HEIGHT + 0*DLG_DEF_SPACE + 1*COMBO_H,
                               NULL, false);
  combo_zoom->setChildColourScheme(PUCLASS_POPUPMENU, dlgCol1[0], dlgCol1[1], dlgCol1[2]);
  combo_zoom->newList(Global::inputDev->ZoomControlStringsGUI);
  combo_zoom->setLabelPlace(PUPLACE_LOWER_LEFT);
  combo_zoom->setLabel(_("Zoom Control"));
  combo_zoom->setCurrentItem(Global::inputDev->zoom_control);
  combo_zoom->reveal();
  combo_zoom->activate();

  // finalize the dialog
  close();
  setSize(COMBO_W + LABEL_W + 2*DLG_DEF_SPACE, 
          4*DLG_DEF_BUTTON_HEIGHT + 4*COMBO_H + 8*DLG_DEF_SPACE + BUTTON_BOX_HEIGHT);
  setCallback(CGUICtrlGeneralCallback);
  centerOnScreen();
  reveal();
  
  // show/hide widgets
  showHideButtons ();
}


/**
 * Destroy the dialog.
 */
CGUICtrlGeneralDialog::~CGUICtrlGeneralDialog()
{
}


/**
 * Update the input method text label
 */
void CGUICtrlGeneralDialog::updateInputMethod()
{
  inputMethodString = _("Input Method: ");
  inputMethodString += Global::inputDev->InputMethodStrings[Global::TXInterface->inputMethod()];
  label_inputMethod->setLabel(inputMethodString.c_str());
}


/** \brief The dialog's callback.
 *
 */
void CGUICtrlGeneralCallback(puObject *obj)
{
  if (obj->getIntegerValue() == CRRC_DIALOG_OK)
  {    
    // Dialog left by clicking OK
    CGUICtrlGeneralDialog* dlg = (CGUICtrlGeneralDialog*)obj;
    
    Global::inputDev->zoom_control = dlg->combo_zoom->getCurrentItem();
  }  
  puDeleteObject(obj);
}


/**
 * Show or hide the buttons depending on the current interface.
 *
 */
void CGUICtrlGeneralDialog::showHideButtons()
{
  int nItem = Global::TXInterface->inputMethod();
  
  if (Global::TXInterface != NULL)
  {
    if (Global::TXInterface->usesMapper())
    {
      button_calibrate->activate();
    }
    else
    {
      button_calibrate->greyOut();
    }

    if (Global::TXInterface->usesMixer())
    {
      button_mixer->activate();
    }
    else
    {
      button_mixer->greyOut();
    }
  }
  else
  {
    button_calibrate->greyOut();
    button_mixer->greyOut();
  }
  
  if ((nItem == T_TX_Interface::eIM_joystick)
        ||
       (nItem == T_TX_Interface::eIM_mouse))
  {
    button_joybuttons->activate();
  }
  else
  {
    button_joybuttons->greyOut();
  }
}

/**
 * Callback activated by the Calibrate button
 */
static void CGUICalibrateButtonCallback(puObject *obj)
{
  if (Global::TXInterface->usesMixer())
  {
    new CGUICalibMapDialog();
  }
}

/**
 * Callback activated by the Device button
 */
static void CGUIDeviceButtonCallback(puObject *obj)
{
  // Create a new dialog. obj is the button, so the object's user data should
  // be a pointer to the dialog.
  new CGUICtrlDeviceDialog((CGUICtrlGeneralDialog*)obj->getUserData());
}

/**
 * Callback activated by the "Buttons" button
 */
static void CGUIJoystickButtonCallback(puObject *obj)
{
  int nItem = Global::TXInterface->inputMethod();

  if (nItem == T_TX_Interface::eIM_mouse)
  {
    new CGUIMouseButtonDialog();
  }
  else
  {
    if (!nJoystickDlg)
    {
      joystickDlg  = new CGUIJoystickDialog();
      nJoystickDlg = 1;
    }
  }
}


/**
 * Callback activated by the "Mixer" button
 */
static void CGUIMixerButtonCallback(puObject *obj)
{
  if (Global::TXInterface->usesMixer())
  {
    new CGUIMixerDialog(Global::TXInterface);
  }
}

