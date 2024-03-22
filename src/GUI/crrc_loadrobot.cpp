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
#include "crrc_loadrobot.h"
#include "../crrc_main.h"
#include "../mod_misc/filesystools.h"
#include "../mod_misc/lib_conversions.h"
#include "util.h"
#include "../robots.h"
#include "../mod_robots/robotfile.h"

#include <iostream>
#include <sys/types.h>
#include <dirent.h>
#include <string>

static void CGUIRobotSelCallback(puObject *obj);
static void CGUIRobotSelPlaneListCallback(puObject *obj);

#define LIST_WIDGET_H       (200)
#define LIST_WIDGET_W       (250)
#define DESCRIPTION_H       (75)
#define DESCRIPTION_W       (LIST_WIDGET_W+300)
#define BUTTON_BOX_HEIGHT   (2*DLG_DEF_SPACE+DLG_DEF_BUTTON_HEIGHT)
#define SCROLL_W            (16)


CGUILoadRobotDialog::CGUILoadRobotDialog() 
            : CRRCDialog()
{
  // height of a text label
  int msg_height  = puGetDefaultLegendFont().getStringHeight("jX")
                  + puGetDefaultLegendFont().getStringDescender()
                  + PUSTR_TGAP + PUSTR_BGAP;
  
  int ymax = LIST_WIDGET_H + DESCRIPTION_H + 1*msg_height + 2*DLG_DEF_SPACE + BUTTON_BOX_HEIGHT;
  int yleft = ymax;
  int yright= ymax;
  int HELP_H = LIST_WIDGET_H + msg_height - DLG_CHECK_H - DLG_DEF_SPACE;
  

  // demo or robot checkbox
  yright -= DLG_DEF_SPACE + DLG_CHECK_H;
  check_demo = new puButton(2*DLG_DEF_SPACE + LIST_WIDGET_W,
                            yright,
                            2*DLG_DEF_SPACE + LIST_WIDGET_W + DLG_CHECK_W,
                            yright + DLG_CHECK_H);
  check_demo->setButtonType(PUBUTTON_VCHECK);
  check_demo->setLabelPlace(PUPLACE_CENTERED_RIGHT);
  check_demo->setLabel(_("Load as demo, replacing user input"));
  check_demo->setValue(0);
  
  // the description box
  yright -= HELP_H + DLG_DEF_SPACE;  
  puaLargeInput* help = new puaLargeInput(2*DLG_DEF_SPACE + LIST_WIDGET_W, yright,
                                          DESCRIPTION_W - LIST_WIDGET_W - DLG_DEF_SPACE, HELP_H,
                                          1,    // num of arrow pairs
                                          SCROLL_W,   // slider width
                                          1);   // wrap text
  help->setText(_("In case you check the box above, this robot will replace your manually controlled airplane.\n"
                "Load an airplane to regain manual control."));
  help->disableInput();
    
  // file selection list
  yleft -=  LIST_WIDGET_H + DLG_DEF_SPACE + msg_height;
  files = new puaFileBox(DLG_DEF_SPACE,
                         yleft,
                         LIST_WIDGET_W,
                         LIST_WIDGET_H, 
                         FileSysTools::getHomePath().c_str(), "crrclog", true);
  files->setLabelPlace(PUPLACE_TOP_LEFT);
  files->setLabel(_("Select file:"));
  files->setUserData(this);
  files->setCallback(CGUIRobotSelPlaneListCallback);
  
  // the description box
  yleft -= DESCRIPTION_H + DLG_DEF_SPACE;
  description = new puaLargeInput(DLG_DEF_SPACE,
                                  yleft,
                                  DESCRIPTION_W,
                                  DESCRIPTION_H,
                                  1,    // num of arrow pairs
                                  16,   // slider width
                                  0);   // wrap text
  description->disableInput();
  description->setText(_("This is a short description of the selected record."));

  // finalize the dialog
  close();
  setSize(DESCRIPTION_W+2*DLG_DEF_SPACE, ymax);
  setCallback(CGUIRobotSelCallback);
  centerOnScreen();
  reveal();
}


/**
 * Destroy the dialog.
 */
CGUILoadRobotDialog::~CGUILoadRobotDialog()
{
}

std::string CGUILoadRobotDialog::getFilename()
{
  const char* path = files->getPath();
  const char* file = files->getStringValue();
  
  if (path != 0 && file != 0)
    return(std::string(path) + "/" + file);
  else
    return("");
}

void CGUILoadRobotDialog::updateFileInfo()
{
  RobotFile* rf = new RobotFile(getFilename());
  description->setText(rf->ReadDescription().c_str());
  delete rf;
}


/**
 *  This callback is invoked when a new file is selected from
 *  the file list
 */
void CGUIRobotSelPlaneListCallback(puObject *obj)
{
  CGUILoadRobotDialog *dlg = (CGUILoadRobotDialog*)obj->getUserData();
  dlg->updateFileInfo();
}


/** \brief The dialog's callback.
 *
 *  Determine if a file was selected and load it.
 */
void CGUIRobotSelCallback(puObject *obj)
{
  CGUILoadRobotDialog *dlg = (CGUILoadRobotDialog*)obj;
    
  if (obj->getIntegerValue() == CRRC_DIALOG_OK)
  {
    if (FileSysTools::fileExists(dlg->getFilename()))
    {
      // Dialog left by clicking OK
      if (dlg->check_demo->getBooleanValue())
      {
        Global::aircraft->setModel(NULL);
        Global::aircraft->loadDemo(dlg->getFilename());
        initialize_flight_model();
      }
      else
      {
        Global::robots->AddRobot(dlg->getFilename());
      }
    }
  }
  
  Global::gui->hide();
  puDeleteObject(obj);
}
