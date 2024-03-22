/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2004, 2005, 2008 Jan Reucker (original author)
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
  

#ifndef CRRC_GUI_DIALOG_H
#define CRRC_GUI_DIALOG_H

#include <plib/pu.h>
#include <vector>

enum
{
  CRRC_DIALOG_CANCEL  = 1,
  CRRC_DIALOG_OK      = 2
};

// some constants for unified dialog style
extern const int DLG_DEF_SPACE;         // space between two widgets
extern const int DLG_DEF_BUTTON_WIDTH;  // width of a puButton
extern const int DLG_DEF_BUTTON_HEIGHT; // height of a puButton
extern const int DLG_CHECK_W;   // width of a puButton (checkbox type)
extern const int DLG_CHECK_H;  // height of a puButton (checkbox type)

class CRRCDialog;


/** \brief The base class for all CRRCsim dialogs.
 *
 *  This class provides a very basic dialog which only
 *  consists of a frame and up to two buttons (OK and/or
 *  Cancel). The frame is just big enough to contain the
 *  two buttons and nothing else.
 *
 *  The class also manages the callbacks for all currently
 *  visible dialogs. Basically all instances of CRRCDialog
 *  and derived classes share only two callbacks. All
 *  dialogs are registered in a static list, so the callbacks
 *  can use the data from the list to determine which dialog
 *  received the event which caused the callback. The
 *  event is then dispatched by activating the dialog's
 *  callback directly.
 *
 *  This somehow complicated mechanism eliminates the need
 *  to provide seperate callbacks for each button in all
 *  derived classes. It also takes care of the fact that
 *  PUI only knows which button's callback was activated,
 *  and not which dialog the button belongs to.
 *
 *  To do something useful with this class you should
 *  derive a new dialog class from CRRCDialog. All you
 *  have to do is the following:
 *
 *  -# Create a derived class.
 *  -# In the derived class' ctor:
 *    - add widgets as needed
 *    - call CRRCDialog::close() (inherited from puDialogBox) after
 *      all widgets have been added
 *    - determine how many space your widgets need and call
 *      CRRCDialog::setSize() to adjust the dialog's frame
 *      (otherwise the widgets will appear outside the frame)
 *    - set a callback for the dialog (see below)
 *    - You might also need to care about the dialog's placement
 *      on screen. By default the dialog is placed horizontally
 *      centered with its lower edge one third from the screen's
 *      top. Use centerOnScreen() if you want to automatically
 *      place the dialog in the middle of the screen.
 *    - reveal() the dialog
 *  -# Provide a callback function for the dialog. The callback
 *     should take care of the data the user entered in the
 *     dialog. It can determine how the user left the dialog
 *     by querying the dialog's integer value (a pointer to
 *     the dialog is provided as the callback's argument). This will
 *     be set to either CRRC_DIALOG_OK or CRRC_DIALOG_CANCEL.
 *     However, <b>the callback must take care of the
 *     dialog's destruction</b> by calling puDeleteObject() on
 *     the dialog object.
 */
class CRRCDialog : public puDialogBox
{
  public:
    puColour dlgCol;
    puColour dlgCol1;

    CRRCDialog( int width = 0, int height = 0,
                int style = CRRC_DIALOG_OK | CRRC_DIALOG_CANCEL);
    virtual ~CRRCDialog();

    virtual void setSize(int w, int h);
    virtual void setPosition(int x, int y);
    virtual void setOKButtonLegend(const char *text);
    virtual void setCancelButtonLegend(const char *text);
  
    virtual bool hasCancelButton();
    virtual bool hasOKButton();
  
    virtual void lock();
    virtual void unlock();
  
    virtual void centerOnScreen();
    virtual void hideOthers();
    virtual void revealAll();
    virtual void setTransparency(float t);

    /** \brief Cyclic update function
     *
     *  This function will be called periodically for the
     *  toplevel dialog. The call is performed instead of
     *  the flight model's idle() call.
     *  Overload it if your dialog needs periodic updates.
     */
    virtual void update() {};
  
    static CRRCDialog* getToplevel();

  private:
    puFrame   *dlgFrame;
    puButton  *butCancel;
    puButton  *butOK;

    void centerButtons();
    static std::vector<CRRCDialog*> instances;

    /** \brief Dummy assignment operator
     *
     *  I assume that this operator isn't needed. To prevent
     *  the compiler from generating a non-working assignment
     *  operator this declaration is needed.
     */
    CRRCDialog& operator=(const CRRCDialog&);

    /** \brief Dummy copy constructor
     *
     *  I assume that there's no need for copying dialogs. To prevent
     *  the compiler from generating a non-working copy
     *  constructor this declaration is needed.
     */
    CRRCDialog(const CRRCDialog&);
};


#endif
