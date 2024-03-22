/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2008 Jens Wilhelm Wulf (original author)
 * Copyright (C) 2008 Jan Reucker
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
#include "crrc_mousebutton.h"

#include <stdio.h>
#include <stdlib.h>

#include "../global.h"
#include "../crrc_main.h"
#include "../mouse_kbd.h"
#include <crrc_config.h>


static void CGUIMouseButtonCallback(puObject *obj);

#define BUTTON_BOX_HEIGHT   (2*DLG_DEF_SPACE+DLG_DEF_BUTTON_HEIGHT)
#define COMBO_W             150
#define COMBO_H             DLG_DEF_BUTTON_HEIGHT
#define LABEL_W             170


CGUIMouseButtonDialog::CGUIMouseButtonDialog()
            : CRRCDialog()
{
  
  combo_l = new puaComboBox(LABEL_W + DLG_DEF_SPACE, BUTTON_BOX_HEIGHT + 4*DLG_DEF_SPACE + 4*COMBO_H,
                            LABEL_W + COMBO_W,       BUTTON_BOX_HEIGHT + 4*DLG_DEF_SPACE + 5*COMBO_H,
                            NULL, false);
  combo_l->setChildColourScheme(PUCLASS_POPUPMENU, dlgCol1[0], dlgCol1[1], dlgCol1[2]);
  combo_l->newList(Global::inputDev->ActionButtonStringsGUI);
  combo_l->setLabelPlace(PUPLACE_LOWER_LEFT);
  combo_l->setLabel(_("Left"));
  combo_l->setCurrentItem(Global::inputDev->mouse_bind_l);
  
  combo_m = new puaComboBox(LABEL_W + DLG_DEF_SPACE, BUTTON_BOX_HEIGHT + 3*DLG_DEF_SPACE + 3*COMBO_H,
                            LABEL_W + COMBO_W,       BUTTON_BOX_HEIGHT + 3*DLG_DEF_SPACE + 4*COMBO_H,
                            NULL, false);
  combo_m->setChildColourScheme(PUCLASS_POPUPMENU, dlgCol1[0], dlgCol1[1], dlgCol1[2]);
  combo_m->newList(Global::inputDev->ActionButtonStringsGUI);
  combo_m->setLabelPlace(PUPLACE_LOWER_LEFT);
  combo_m->setLabel(_("Middle"));
  combo_m->setCurrentItem(Global::inputDev->mouse_bind_m);
  
  combo_r = new puaComboBox(LABEL_W + DLG_DEF_SPACE, BUTTON_BOX_HEIGHT + 2*DLG_DEF_SPACE + 2*COMBO_H,
                            LABEL_W + COMBO_W,       BUTTON_BOX_HEIGHT + 2*DLG_DEF_SPACE + 3*COMBO_H,
                            NULL, false);
  combo_r->setChildColourScheme(PUCLASS_POPUPMENU, dlgCol1[0], dlgCol1[1], dlgCol1[2]);
  combo_r->newList(Global::inputDev->ActionButtonStringsGUI);
  combo_r->setLabelPlace(PUPLACE_LOWER_LEFT);
  combo_r->setLabel(_("Right"));
  combo_r->setCurrentItem(Global::inputDev->mouse_bind_r);
  
  combo_up = new puaComboBox(LABEL_W + DLG_DEF_SPACE, BUTTON_BOX_HEIGHT + 1*DLG_DEF_SPACE + 1*COMBO_H,
                             LABEL_W + COMBO_W,       BUTTON_BOX_HEIGHT + 1*DLG_DEF_SPACE + 2*COMBO_H,
                             NULL, false);
  combo_up->setChildColourScheme(PUCLASS_POPUPMENU, dlgCol1[0], dlgCol1[1], dlgCol1[2]);
  combo_up->newList(Global::inputDev->ActionButtonStringsGUI);
  combo_up->setLabelPlace(PUPLACE_LOWER_LEFT);
  combo_up->setLabel(_("Wheel up"));
  combo_up->setCurrentItem(Global::inputDev->mouse_bind_u);
  
  combo_down = new puaComboBox(LABEL_W + DLG_DEF_SPACE, BUTTON_BOX_HEIGHT + 0*DLG_DEF_SPACE + 0*COMBO_H,
                               LABEL_W + COMBO_W,       BUTTON_BOX_HEIGHT + 0*DLG_DEF_SPACE + 1*COMBO_H,
                               NULL, false);
  combo_down->setChildColourScheme(PUCLASS_POPUPMENU, dlgCol1[0], dlgCol1[1], dlgCol1[2]);
  combo_down->newList(Global::inputDev->ActionButtonStringsGUI);
  combo_down->setLabelPlace(PUPLACE_LOWER_LEFT);
  combo_down->setLabel(_("Wheel down"));
  combo_down->setCurrentItem(Global::inputDev->mouse_bind_d);
  
  // Only show the up and down combos if SDL is capable of handling mouse buttons.
#ifdef SDL_WITHOUT_MOUSEWHEEL
  combo_up->hide();
  combo_down->hide();
#endif
  
  // finalize the dialog
  close();
  setSize(COMBO_W + LABEL_W + 2*DLG_DEF_SPACE, 
          5*COMBO_H + 5*DLG_DEF_SPACE + BUTTON_BOX_HEIGHT);
  setCallback(CGUIMouseButtonCallback);
  centerOnScreen();
  reveal();
}


/**
 * Destroy the dialog.
 */

CGUIMouseButtonDialog::~CGUIMouseButtonDialog()
{
}


/** \brief The dialog's callback.
 *
 */
void CGUIMouseButtonCallback(puObject *obj)
{
  if (obj->getIntegerValue() == CRRC_DIALOG_OK)
  {    
    // Dialog left by clicking OK
    CGUIMouseButtonDialog* dlg = (CGUIMouseButtonDialog*)obj;
    
    Global::inputDev->mouse_bind_l = dlg->combo_l->getCurrentItem();
    Global::inputDev->mouse_bind_m = dlg->combo_m->getCurrentItem();
    Global::inputDev->mouse_bind_r = dlg->combo_r->getCurrentItem();
    Global::inputDev->mouse_bind_u = dlg->combo_up->getCurrentItem();
    Global::inputDev->mouse_bind_d = dlg->combo_down->getCurrentItem();
  }
  
  puDeleteObject(obj);
}
