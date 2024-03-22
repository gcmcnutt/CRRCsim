/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2010 Jens Wilhelm Wulf (original author)
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
#include "../aircraft.h"
#include "crrc_gui_main.h"
#include "crrc_setrecordname.h"
#include "../crrc_main.h"
#include "../mod_misc/lib_conversions.h"
#include "util.h"
#include "../record.h"

#include <iostream>
#include <sys/types.h>
#include <dirent.h>
#include <string>

static void CGUISetRecordnameCallback(puObject *obj);

#define ITEM_W              (500)
#define SLIDER_H            (DLG_DEF_BUTTON_HEIGHT)
#define HELP_H              (140)
#define DESCRIPTION_H       (75)
#define BUTTON_BOX_HEIGHT   (2*DLG_DEF_SPACE+DLG_DEF_BUTTON_HEIGHT)
#define SCROLL_W            (16)

CGUISetRecordNameDialog::CGUISetRecordNameDialog() 
            : CRRCDialog()
{
  // height of a text label
  int msg_height  = puGetDefaultLegendFont().getStringHeight("jX")
                  + puGetDefaultLegendFont().getStringDescender()
                  + PUSTR_TGAP + PUSTR_BGAP;

  int ymax = HELP_H + DESCRIPTION_H + SLIDER_H + 2*msg_height + 3*DLG_DEF_SPACE + BUTTON_BOX_HEIGHT ;
  int y = ymax;

  // the description box
  y -= HELP_H + DLG_DEF_SPACE;
  puaLargeInput* help = new puaLargeInput(DLG_DEF_SPACE, y,
                                          ITEM_W, HELP_H,
                                          1,    // num of arrow pairs
                                          SCROLL_W,   // slider width
                                          1);   // wrap text
  help->setText(_("Both filename and description become effective when the flight log is finally closed (on flight reset, change of airplane/location, exit).\n"
                "Without a name, the log will not be saved!\n"
                "Your best F3F runs are named (and therefore saved) automatically.\n"
                "Replay flight logs using 'Robots | Load Robot'."));
  help->disableInput();
  
  
  y -= SLIDER_H + DLG_DEF_SPACE + msg_height;
  filename = new puInput(DLG_DEF_SPACE,         y,
                         DLG_DEF_SPACE+ 200, y + SLIDER_H);
  filename->setLabelPlace(PUPLACE_TOP_LEFT);
  filename->setLabel(_("Filename:"));
  filename->setValue(Global::recorder->GetFilename().c_str());
  filename->setUserData(this);
  
  // the description box
  y -= DESCRIPTION_H + DLG_DEF_SPACE + msg_height;
  description = new puaLargeInput(DLG_DEF_SPACE, y,
                                  ITEM_W, DESCRIPTION_H,
                                  1,    // num of arrow pairs
                                  SCROLL_W,   // slider width
                                  1);   // wrap text
  description->setLabelPlace(PUPLACE_TOP_LEFT);
  description->setLabel(_("Description:"));
  description->setText(Global::recorder->descr.c_str());

  // finalize the dialog
  close();
  setSize(ITEM_W + 2*DLG_DEF_SPACE, ymax);
  setCallback(CGUISetRecordnameCallback);
  centerOnScreen();
  reveal();
}


/**
 * Destroy the dialog.
 */
CGUISetRecordNameDialog::~CGUISetRecordNameDialog()
{
}


/** \brief The dialog's callback.
 *
 */
void CGUISetRecordnameCallback(puObject *obj)
{
  CGUISetRecordNameDialog *dlg = (CGUISetRecordNameDialog*)obj;
    
  if (obj->getIntegerValue() == CRRC_DIALOG_OK)
  {
    Global::recorder->descr = dlg->description->getText();
    Global::recorder->SetFilename(dlg->filename->getStringValue());
  }
  
  Global::gui->hide();
  puDeleteObject(obj);
}
