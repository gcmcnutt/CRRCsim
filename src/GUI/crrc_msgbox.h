/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2004, 2005 Jan Reucker (original author)
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
  

// crrc_gui_msgbox - A simple message box.

#ifndef CRRC_GUI_MSGBOX_H
#define CRRC_GUI_MSGBOX_H

#include <plib/pu.h>
#include <vector>
#include <string>

#include "crrc_dialog.h"

class CGUIMsgBox;

/** \brief A simple message box.
 *
 *  This class provides a simple message box which can
 *  be used to inform the user with a text string. When
 *  constructed with the default parameters, the box
 *  only contains one line of text and an OK button to
 *  leave the box.
 *
 *  Example:<br>
 *  <code>
 *  void NIY_func()<br>
 *  {<br>
 *  &nbsp;&nbsp;new CGUIMsgBox("Not implemented yet!");<br>
 *  }<br>
 *  </code>
 *
 *  That's all. Destruction of the box is automatically
 *  handled by the internal callback.
 *
 *  To construct a query box one can provide two extra
 *  parameters to the constructor: the desired buttons
 *  (CRRC_DIALOG_OK / CRRC_DIALOG_CANCEL) and the name
 *  of a callback function which receives the dialog
 *  result. As with the simple default message box,
 *  one does not need to care about the destruction
 *  of the dialog. It is automatically destroyed after
 *  the callback function returned.
 *
 *  The callback's type must be a function which receives
 *  an int parameter and returns void. The parameter will
 *  be set to CRRC_DIALOG_OK or CRRC_DIALOG_CANCEL, depending
 *  on which button the user selected in the dialog.
 *
 *  Example:<br>
 *  <code>
 *  void getResult(int res)<br>
 *  {<br>
 *  &nbsp;&nbsp;if (res == CRRC_DIALOG_OK)<br>
 *  &nbsp;&nbsp;&nbsp;&nbsp;say_hello();<br>
 *  }<br>
 *  &nbsp;<br>
 *  void ask_user()<br>
 *  {<br>
 *  &nbsp;&nbsp;CGUIMsgBox *msg = new CGUIMsgBox("Say hello?",<br>
 *  &nbsp;&nbsp;&nbsp;&nbsp;CRRC_DIALOG_OK | CRRC_DIALOG_CANCEL,<br>
 *  &nbsp;&nbsp;&nbsp;&nbsp;getResult);<br>
 *  &nbsp;&nbsp;msg->setOKButtonLegend("Yes");<br>
 *  &nbsp;&nbsp;msg->setCancelButtonLegend("No");<br>
 *  }<br>
 *  </code>
 */
class CGUIMsgBox : public CRRCDialog
{
  public:
    CGUIMsgBox( const char *msg,
                int buttons = CRRC_DIALOG_OK,
                void (*selectCallback)(int) = NULL);
    void setSelectCallback(void (*)(int));
    void activateCallback();

  private:
   /**
    * The message is stored internally, because it needs to be persistent
    * after the construction -- obviously only the pointer to the message 
    * is stored. So there is a need to keep it.
    */
    std::string myMessage;
    void (*selCallback)(int);
};

/*****
*
* Simple message box without any button for display of a waiting mesage 
*
* \param msg The message to be displayed in the box.
******/
class CGUIWaitingBox
  {
  public:
    CGUIWaitingBox( const char *msg);
    ~CGUIWaitingBox();
  private:
    puPopup * dbox;
  };
#endif // CRRC_GUI_MSGBOX_H
