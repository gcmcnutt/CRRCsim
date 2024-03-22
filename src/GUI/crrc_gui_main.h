/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2004, 2005, 2008 Jan Reucker (original author)
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
  

#ifndef CRRC_GUIMAIN_H
#define CRRC_GUIMAIN_H

#include <SDL.h>
#include <plib/pu.h>

#include "../mod_fdm/fdm_inputs.h"

#define N_COMPASS 10

/** \brief The main GUI object
 *
 *  This object provides the framework for the CRRCsim GUI. It
 *  contains a single menu widget and handles UI drawing and 
 *  incoming SDL events.
 */
class CGUIMain
{
  public:
    CGUIMain(bool vis = true);
    ~CGUIMain();
    
    bool keyDownEventHandler(SDL_keysym& key);
    bool keyUpEventHandler(SDL_keysym& key);
    bool mouseButtonDownHandler(int btn, int x, int y);
    bool mouseButtonUpHandler(int btn, int x, int y);
    bool mouseMotionHandler(int x, int y);

    void reveal();
    void hide();

    void draw();

    /** \brief Dummy mouse motion handler. 
     *
     *  Always returns false as PUI does not evaluate the
     *  mouse motion.
     *  \param state not used
     *  \param x not used
     *  \param y not used
     *  \return Always false.
     */
    inline bool mouseMotionHandler(char state, int x, int y)
    {
      return false;
    };

    /** \brief Convert SDL mouse buttons to PUI mouse buttons.
     *
     *  This inline function simply returns its argument
     *  minus one: SDL counts mouse buttons from 1, PUI
     *  counts from 0.
     *  \param btn Mouse button in SDL numeration.
     *  \return Mouse button in PUI numeration.
     */
    inline int translateMouse(int btn){return (btn-1);};

    /** \brief Determine if the GUI is currently visible.
      *
      * \return true if the GUI is visible, false otherwise.
      */
    inline bool isVisible() {return visible;};
   
    /**
     * Set text of verbose output
     */
    void setVerboseText(const char* msg);
   
    /**
     * Draw or hide the HUD compass labels
     */
    void doHUDCompass(const float field_of_view);
   
    /**
     * Show error message
     */
    void errorMsg(const char* message);

    void doQuitDialog();
    
    /**
     *  Propagate the simulation control input values
     *  to the GUI.
     */
    void setInputValues(TSimInputs *in);

    /**
    * The GUI's "idle" function.
    **/
    void GUI_IdleFunction(TSimInputs *in);

    /**
     *  Access the local copy of the control input values.
     */
    TSimInputs * getInputValues();

  private:
    int translateKey(const SDL_keysym& keysym);
   
    bool        visible;
   
    puMenuBar   *main_menu_bar;
    puText*     verboseOutput;
    TSimInputs  input;

    fntTexFont *VerbosityFont;

    static const int  nCompass = N_COMPASS;
    puText*           compass_x[N_COMPASS+1];
    puText*           compass_y[N_COMPASS+1];
    std::string       compass_msg_x[N_COMPASS+1];
    std::string       compass_msg_y[N_COMPASS+1];
};


#endif // CRRC_GUIMAIN_H

