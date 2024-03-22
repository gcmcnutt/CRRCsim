/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2004, 2005, 2007 Jan Reucker (original author)
 * Copyright (C) 2005, 2008 Jens Wilhelm Wulf
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
  

// implementation of class CGUIMsgBox

#include "crrc_msgbox.h"

#include <iostream>

static void CGUIMsgBoxCallback(puObject *obj);

/** \brief Create the message box
 *
 *  This constructor creates a dialog box which consists of
 *  the message string <code>msg</code> and an OK button.
 *  \param msg The message to be displayed in the box.
 */
CGUIMsgBox::CGUIMsgBox(const char *msg,
                       int buttons,
                       void (*selectCallback)(int))
            : CRRCDialog(0, 0, buttons), selCallback(selectCallback)
{
  // calculate sizes and position
  int msg_width, msg_height;
  int total_width, total_height;
  int line_height;
  
  line_height = getLabelFont().getStringHeight("X");

  msg_width = puGetDefaultLabelFont().getStringWidth(msg)
              + PUSTR_LGAP + PUSTR_RGAP;
  msg_height = puGetDefaultLabelFont().getStringHeight(msg)
               + puGetDefaultLabelFont().getStringDescender()
               + PUSTR_TGAP + PUSTR_BGAP;
  // std::cout << "string width " << puGetDefaultLabelFont().getStringWidth(msg) << "   msg_width " << msg_width << std::endl;
  // std::cout << "string height " << puGetDefaultLabelFont().getStringHeight(msg) 
  //           << "   desc hgt " << puGetDefaultLabelFont().getStringDescender()
  //           << "   msg_height " << msg_height 
  //           << std::endl;

  // 10 px space, string, 10 pixel space, 25 px button, 10 px space
  total_height = DLG_DEF_SPACE + msg_height + DLG_DEF_SPACE
                 + DLG_DEF_BUTTON_HEIGHT + DLG_DEF_SPACE;
  total_width = (msg_width < DLG_DEF_BUTTON_WIDTH ? DLG_DEF_BUTTON_WIDTH : msg_width)
                + 2 * DLG_DEF_SPACE;

  // Create a text widget
  puText *txt = new puText(total_width/2 - msg_width/2,
                           total_height - DLG_DEF_SPACE - line_height);
  myMessage = msg;
  txt->setLabel(myMessage.c_str());

  // no more widgets, close the underlying puDialogBox
  close();

  // set the dialog's callback
  setCallback(CGUIMsgBoxCallback);

  // resize the dialog
  int current_width = getBBox()->max[0] - getBBox()->min[0];
  int current_height = getBBox()->max[1] - getBBox()->min[1];
  current_width = (current_width > total_width) ? current_width:total_width;
  current_height += 2 * DLG_DEF_SPACE + line_height;
  setSize(current_width, current_height);

  // center the dialog on screen
  centerOnScreen();

  // show the dialog
  reveal();
}

/** \brief Set a callback.
 *
 *  This is only needed if you construct a box with
 *  more than one button, although it will only work
 *  for a one-button-box.
 */
void CGUIMsgBox::setSelectCallback(void (*cb)(int))
{
  selCallback = cb;
}

/** \brief Invoke the custom callback.
 *
 *  Invokes the box's callback (if a callback was
 *  configured).
 */
void CGUIMsgBox::activateCallback()
{
  if (selCallback != NULL)
  {
    selCallback(getIntegerValue());
  }
}

static void CGUIMsgBoxCallback(puObject *obj)
{
  CGUIMsgBox *dlg = (CGUIMsgBox*)obj;

  dlg->activateCallback();
  puDeleteObject(obj);
}


/*****
*
* Simple message box without any button for display of a waiting mesage 
*
******/
CGUIWaitingBox::CGUIWaitingBox(const char *msg)
  {
  /** \brief Create the message box
  *  \param msg The message to be displayed in the box.
  */
  int msg_width, total_width;
  int total_height=200;//for visibility
  msg_width = puGetDefaultLabelFont().getStringWidth(msg) + PUSTR_LGAP + PUSTR_RGAP;
  total_width = msg_width + 2 * DLG_DEF_SPACE;

  dbox = new puPopup (total_width, total_height);
  new puFrame ( 0, 0, total_width, total_height ) ;
  puText *ltxt = new puText  ( DLG_DEF_SPACE,total_height/2 );
  ltxt -> setLabel ( msg);
	
  // center the dialog on screen
  int wwidth, wheight;
  puGetWindowSize(&wwidth, &wheight);
  dbox->setPosition(wwidth/2 - total_width/2, wheight/2 - total_height/2 );
  
  // close the underlying puDialogBox
  dbox->close();

  // show the dialog
  dbox->reveal();
  }
//destructor
CGUIWaitingBox::~CGUIWaitingBox()
  {
  if(dbox) delete dbox;
  }

