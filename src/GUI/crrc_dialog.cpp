/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2004, 2005, 2007, 2008 Jan Reucker (original author)
 * Copyright (C) 2008 Jens Wilhelm Wulf
 *               2012 Joel Lienard
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
#include "crrc_dialog.h"

#include <iostream>

/**
 *  Constants for unified dialog style.
 */
const int DLG_DEF_SPACE         = 10;
const int DLG_DEF_BUTTON_WIDTH  = 70;
const int DLG_DEF_BUTTON_HEIGHT = 25;
const int DLG_CHECK_W = 15;
const int DLG_CHECK_H = 15;

#define DLG_COLOR_SHIFT 		    0.1

/** \brief A list of all currently existing CRRCDialogs.
 *
 *  This vector keeps a pointer to each instance of CRRCDialog.
 *  We need it to determine the dialog which shall receive
 *  a click after a button callback was activated. PLIB only
 *  provides a pointer to the actual button, but we need
 *  a pointer to the dialog which contains the button.
 */
std::vector<CRRCDialog*> CRRCDialog::instances;


// Prototypes for this file
static void CRRCDialogOKCallback(puObject* obj);
static void CRRCDialogCancelCallback(puObject* obj);
static void CRRCDialogCallback(puObject *obj);


/** \brief Construct the dialog frame and up to two buttons.
 *
 *  The created dialog will be at least big enough to hold
 *  two buttons and can be enlarged by providing the
 *  width and height arguments.
 *  \param width Dialog's width (must be greater than the space
 *               occupied by the buttons to take effect)
 *  \param height Dialog's height (must be greater than the space
 *                occupied by the buttons to take effect)
 *  \param style Can be CRRC_DIALOG_OK (dialog has an OK button),
 *               CRRC_DIALOG_CANCEL (dialog has a Cancel button)
 *               or both (bitwise OR'ed, which also
 *               is the default).
 */
CRRCDialog::CRRCDialog(int width, int height, int style)
    : puDialogBox(10, 10),
    butCancel(NULL), butOK(NULL)
{
  // calculate sizes and position
  int min_width, min_height;
  int window_width, window_height;

  // Calculate minimum sizes:
  // one button and two spaces high
  min_height = 2 * DLG_DEF_SPACE + DLG_DEF_BUTTON_HEIGHT;
  // two buttons and 3 spaces wide
  min_width = 2 * DLG_DEF_BUTTON_WIDTH + 3 * DLG_DEF_SPACE;

  width = (min_width > width) ? min_width : width;
  height = (min_height > height) ? min_height : height;

  // Place message box on screen: horiz. centered, 1/3 from top
  puGetWindowSize(&window_width, &window_height);
  setPosition(window_width/2 - width/2,
              window_height*2/3 - height/2);

  // Create the dialog's frame
  dlgFrame = new puFrame(0, 0, width, height);

  // Just make sure nothing weird happens if anyone calls this
  // ctor with an invalid style parameter --> fall back to default
  if (!(style & CRRC_DIALOG_OK) && !(style & CRRC_DIALOG_CANCEL))
  {
    style = CRRC_DIALOG_OK | CRRC_DIALOG_CANCEL;
  }

  // Create buttons if wanted
  if (style & CRRC_DIALOG_OK)
  {
    butOK = new puOneShot(0, DLG_DEF_SPACE,
                          DLG_DEF_BUTTON_WIDTH,
                          DLG_DEF_SPACE + DLG_DEF_BUTTON_HEIGHT);
    butOK->setCallback(CRRCDialogOKCallback);
    butOK->setLegend(_("OK"));
    butOK->setUserData(this);
    // makeReturnDefault lets the button look ugly in SMALL_BEVELLED...
    //butOK->makeReturnDefault(true);
  }
  if (style & CRRC_DIALOG_CANCEL)
  {
    butCancel = new puOneShot(0, DLG_DEF_SPACE,
                              DLG_DEF_BUTTON_WIDTH,
                              DLG_DEF_SPACE + DLG_DEF_BUTTON_HEIGHT);
    butCancel->setCallback(CRRCDialogCancelCallback);
    butCancel->setLegend(_("Cancel"));
    butCancel->setUserData(this);
  }
  // Place the buttons.
  centerButtons();

  // get default dialog color and define color options 
  // to be used by other items (e.g. popup menu)
  getColour(PUCOL_FOREGROUND, &dlgCol[0], &dlgCol[1], &dlgCol[2], &dlgCol[3]);
  puSetColour(dlgCol1, dlgCol);
  dlgCol1[0] += dlgCol1[0] < 1. - DLG_COLOR_SHIFT ? DLG_COLOR_SHIFT : 1. - dlgCol1[0];
  dlgCol1[1] += dlgCol1[1] < 1. - DLG_COLOR_SHIFT ? DLG_COLOR_SHIFT : 1. - dlgCol1[1];
  dlgCol1[2] += dlgCol1[2] < 1. - DLG_COLOR_SHIFT ? DLG_COLOR_SHIFT : 1. - dlgCol1[2];

  // Set the deault dialog's callback, likely replaced by user
  setCallback(CRRCDialogCallback);
  
  CRRCDialog::instances.push_back(this);
  reveal();
}


/** \brief Destroy the object.
 *
 *  Deallocates the object's dynamic memory and removes
 *  the dialog from the internal list of all dialog instances.
 */
CRRCDialog::~CRRCDialog()
{
  // Delete dialog from internal list of instances
  std::vector<CRRCDialog*>::iterator it = CRRCDialog::instances.begin();
  while(it != CRRCDialog::instances.end())
  {
    if (*it == this)
    {
      instances.erase(it);
      break;
    }
    it++;
  }

  puDeleteObject(butOK);
  puDeleteObject(butCancel);
}

void CRRCDialog::hideOthers()
{
  std::vector<CRRCDialog*>::iterator it = CRRCDialog::instances.begin();
  while(it != CRRCDialog::instances.end())
  {
    if (*it != this)
    {
      CRRCDialog* d = *it;
      d->hide();
    }
    it++;
  }
}
/** \brief Set visible all the dialogues
 */
void CRRCDialog::revealAll()
{
  std::vector<CRRCDialog*>::iterator it = CRRCDialog::instances.begin();
  while(it != CRRCDialog::instances.end())
  {
    CRRCDialog* d = *it;
    if( !(d->isVisible() )) d->reveal();
  it++;
  }
}
/** \brief Set the dialog's transparency.
 *  \param t  transparency: 0 (opaque) to 1 (full trasparent).
 */
void CRRCDialog::setTransparency(float t)
{
  float r, g, b, a;
  a = 1 - t;
  dlgFrame->getColour ( PUCOL_FOREGROUND, &r, &g, &b);
  dlgFrame->setColour ( PUCOL_FOREGROUND, r, g, b, a ) ;
}


/** \brief Set the dialog's size.
 *
 *  This method overloads the inherited puObject::setSize() method.
 *  In addition to resizing the base object it resizes the
 *  underlying puFrame.
 *  \param w  The dialog's new width.
 *  \param h  The dialog's new height.
 */
void CRRCDialog::setSize(int w, int h)
{
  puDialogBox::setSize(w, h);
  dlgFrame->setSize(w, h);
  centerButtons();
}


/** \brief Place the dialog on screen.
 *
 *  Make sure that the dialog isn't placed off-screen, then
 *  just delegate to our parent's setPosition method.
 */
void CRRCDialog::setPosition(int x, int y)
{
  if (x < 0)
  {
    x = 0;
  }
  if (y < 0)
  {
    y = 0;
  }
  puDialogBox::setPosition(x, y);
}


/** \brief Re-center the dialog's buttons.
 *
 *  If both buttons are activated, the following layout is applied:
 *  The "OK" button will be placed DLG_DEF_SPACE/2 pixels
 *  left of the dialog's horizontal center. The dialog's
 *  Cancel button will be placed DLG_DEF_SPACE/2 pixels
 *  right of the center.
 *
 *  If only one button is activated it will be centered.
 *
 *  In both cases, the buttons will be placed DLG_DEF_SPACE
 *  pixels from the dialog's bottom edge.
 */
void CRRCDialog::centerButtons()
{
  int w, h, ok_x, ok_y = 0, can_x, can_y = 0;
  int buttons = 0;

  puDialogBox::getSize(&w, &h);
  if (butOK != NULL)
  {
    buttons++;
    butOK->getPosition(&ok_x, &ok_y);
  }
  if (butCancel != NULL)
  {
    buttons++;
    butCancel->getPosition(&can_x, &can_y);
  }

  switch (buttons)
  {
    case 1:
      // only one button --> centered
      ok_x = can_x = (w - DLG_DEF_BUTTON_WIDTH) / 2;
      break;

    case 2:
    default:
      // two buttons
      ok_x  = (w - DLG_DEF_SPACE)/2 - DLG_DEF_BUTTON_WIDTH;
      can_x = (w + DLG_DEF_SPACE)/2;
      break;
  }

  if (butOK != NULL)
  {
    butOK->setPosition(ok_x, ok_y);
  }
  if (butCancel != NULL)
  {
    butCancel->setPosition(can_x, can_y);
  }
}


/** \brief Center the dialog on screen
 *
 *  This method tries to place the dialog exactly in the middle of
 *  the screen.
 *
 *  It doesn't work well, as combo boxes, when expanded upwards,
 *  seem to influence getABox -- but also getBBox, so using that doesn't help.
 */
void CRRCDialog::centerOnScreen()
{
  int wwidth, wheight;
  int current_width = getABox()->max[0] - getABox()->min[0];
  int current_height = getABox()->max[1] - getABox()->min[1];

  puGetWindowSize(&wwidth, &wheight);
  setPosition(wwidth/2 - current_width/2, wheight/2 - current_height/2);
}


/** \brief The generic dialog callback.
 *
 *  This callback is invoked if one of the dialog's button is
 *  activated. It simply close the dialog.
 */
void CRRCDialogCallback(puObject* obj)
{
  puDeleteObject(obj);
}


/** \brief The generic OK callback.
 *
 *  This callback is invoked if the dialog's OK button is
 *  activated. It searches for the dialog the button belongs
 *  to, sets the dialog's value to CRRC_DIALOG_OK and
 *  invokes the dialog's callback.
 */
void CRRCDialogOKCallback(puObject* obj)
{
  CRRCDialog *dlg = static_cast<CRRCDialog*>(obj->getUserData());
  dlg->setValue(CRRC_DIALOG_OK);
  dlg->invokeCallback();
}


/** \brief The generic Cancel callback.
 *
 *  This callback is invoked if the dialog's Cancel button is
 *  activated. It searches for the dialog the button belongs
 *  to, sets the dialog's value to CRRC_DIALOG_OK and
 *  invokes the dialog's callback.
 */
void CRRCDialogCancelCallback(puObject* obj)
{
  CRRCDialog *dlg = static_cast<CRRCDialog*>(obj->getUserData());
  dlg->setValue(CRRC_DIALOG_CANCEL);
  dlg->invokeCallback();
}


/** \brief Set the OK button legend to a different text.
 *
 *  This method is useful if the default "OK"/"Cancel"
 *  button pair does not suit your needs, e.g. when you
 *  need "Yes"/"No" buttons.
 */
void CRRCDialog::setOKButtonLegend(const char *text)
{
  butOK->setLegend(text);
}


/** \brief Set the Cancel button legend to a different text.
 *
 *  This method is useful if the default "OK"/"Cancel"
 *  button pair does not suit your needs, e.g. when you
 *  need "Yes"/"No" buttons.
 */
void CRRCDialog::setCancelButtonLegend(const char *text)
{
  butCancel->setLegend(text);
}


/** \brief Get a pointer to the top-most dialog on screen.
 *
 *  This method returns a pointer to the top-most dialog
 *  on screen, or NULL if no dialog is visible.
 */
CRRCDialog* CRRCDialog::getToplevel()
{
  CRRCDialog *ret = NULL;

  if (instances.size() > 0)
  {
    ret = instances.back();
  }

  return ret;
}


/** \brief Determine if the dialog has an OK button.
 *
 *
 */
bool CRRCDialog::hasOKButton()
{
  bool ret = false;
  
  if (butOK != NULL)
  {
    ret = true;
  }
  
  return ret;
}

/** \brief Determine if the dialog has a Cancel button.
 *
 *
 */
bool CRRCDialog::hasCancelButton()
{
  bool ret = false;
  
  if (butCancel != NULL)
  {
    ret = true;
  }
  
  return ret;
}

/** \brief Lock the dialog.
 *
 *  Greys out the buttons so the user can't close the
 *  dialog.
 */
void CRRCDialog::lock()
{
  if (butOK != NULL)
  {
    butOK->greyOut();
  }
  if (butCancel != NULL)
  {
    butCancel->greyOut();
  }
}

/** \brief Unlock the dialog.
 *
 *  Reactivates the dialog's buttons.
 */
void CRRCDialog::unlock()
{
  if (butOK != NULL)
  {
    butOK->activate();
  }
  if (butCancel != NULL)
  {
    butCancel->activate();
  }
}
