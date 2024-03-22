/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2010 Joel Lienard (original author)
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
  

/*
 * crrc_vewwind.h
 * classe  CGUIViewWindDialog
 * visualisation of slope and wind profils, for debug purpose
 */

#ifndef CRRC_GUIVIEWWIND_H
#define CRRC_GUIVIEWWIND_H

#include <plib/pu.h>
#include <plib/puAux.h>

#include "crrc_dialog.h"
#include "crrc_slider.h"
#include "../mod_misc/SimpleXMLTransfer.h"

class CGUIViewWindDialog;


class CGUIViewWindDialog : public CRRCDialog
{
  public:
    CGUIViewWindDialog();
    ~CGUIViewWindDialog();
    void viewWind2D(puObject *obj, int dx, int dy, int color_mode);
    
    puFrame *graphe;
    puButtonBox *buttonbox_wind, *buttonbox_color;
    crrcSlider *slider_position, *slider_direction, *slider_width, *slider_hoffs, *slider_voffs;
    int direction, position, width, hoffs, voffs;
    int wind_mode, color_mode;
  
  private:
    void setColorFromScale(float val, float min, float max);
    void arrow2D(float x, float y, float dx, float dy, float lg);
};

#endif // CRRC_GUIVIEWWIND_H
