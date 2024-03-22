/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2008 Jan Reucker (original author)
 * Copyright (C) 2008 Jens Wilhelm Wulf
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
  

// crrc_calibmap - Dialog for calibration and mapping

#ifndef CRRC_CALIBMAP_H
#define CRRC_CALIBMAP_H

#include <plib/puAux.h>
#include "crrc_dialog.h"
#include "../mod_inputdev/inputdev.h"

// NUM_AXISFUNCS includes the dummy axis 0 ("NOTHING"), so we use one less
#define NUM_FUNCTIONS   (T_AxisMapper::NUM_AXISFUNCS - 1)

/** \brief Dialog for calibration and mapping
 *
 *  This dialog configures the T_AxisMapper and
 *  T_Calibration children of a T_TX_Interface.
 */
class CGUICalibMapDialog : public CRRCDialog
{
  public:
    CGUICalibMapDialog();
    ~CGUICalibMapDialog();

   /**
    * update the dialog if something has changed
    */
    void update();
   
    int         state;
    int         numaxis;
    float       minval[TX_MAXAXIS];
    float       maxval[TX_MAXAXIS];
    int sVerbosity; // save vebosity level
    
    puButton      *butCalibrate;
    puaSelectBox  *axismapping[NUM_FUNCTIONS];
    puButton      *invert[NUM_FUNCTIONS];
    puaComboBox   *combo_radiotype;
    puGroup       *calibgroup;
    puGroup       *mapgroup;

    void save_mapping (void);
   
  private:
    puSlider    *axis[TX_MAXAXIS];
    char        *axislabel[TX_MAXAXIS+1];
    const char  *selectlist[TX_MAXAXIS+2];  
};



#endif // CRRC_CALIBMAP_H
