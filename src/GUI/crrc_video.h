/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2007, 2008 Jens Wilhelm Wulf (original author)
 * Copyright (C) 2005, 2006, 2008 Jan Reucker
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
  

// crrc_VIDEO.h - Airplane selection dialog

#ifndef CRRC_VIDEO_H
#define CRRC_VIDEO_H

#include <plib/pu.h>
#include <plib/puAux.h>

#include "crrc_dialog.h"
#include "crrc_slider.h"

class CGUIVideoDialog;

/** \brief The video options dialog.
 *
 */
class CGUIVideoDialog : public CRRCDialog
{
  public:
    CGUIVideoDialog();
    ~CGUIVideoDialog();

   
    puaComboBox* combo_res;
    puButton*   fs_check;
    crrcSlider* slider_autozoom;
    crrcSlider* slider_smartcam;
    crrcSlider* slider_sloppycam;
    crrcSlider* slider_fps;
    crrcSlider* slider_texoff;

    int full_x;
    int full_y;
    int window_x;
    int window_y;
    int use_fs;
    int use_smart;
    float dOldSkyTexOff;
    float flSmartCam;
    float flSloppyCam;
    
    void    updateListeResolutions(int fsuse);
private:
    char**  resolutions;
    int     nResolutions;
    void    freeListeResolutions(void);
};
#endif // CRRC_VIDEO_H
