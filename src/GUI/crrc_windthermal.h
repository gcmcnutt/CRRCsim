/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2006, 2008 Jens Wilhelm Wulf (original author)
 * Copyright (C) 2008 Jan Reucker
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
  

// crrc_WINDTHERMAL.h - Airplane selection dialog

#ifndef CRRC_WINDTHERMAL_H
#define CRRC_WINDTHERMAL_H

#include <plib/pu.h>
#include <plib/puAux.h>

#include "crrc_dialog.h"
#include "crrc_slider.h"
#include "../mod_misc/SimpleXMLTransfer.h"

class CGUIWindThermalDialog;

/** \brief The WindThermal options dialog.
 *
 */
class CGUIWindThermalDialog : public CRRCDialog
{
  public:
    CGUIWindThermalDialog();
    ~CGUIWindThermalDialog();
   
    puaComboBox*       comboPresetsWind;
    char**             presetsWind;
    SimpleXMLTransfer* presetGrpWind;
    int                nPresetsWind;
   
    puaComboBox*       comboPresetsThermal;
    char**             presetsThermal;
    SimpleXMLTransfer* presetGrpThermal;
    int                nPresetsThermal;

    crrcSlider*        slider_windVelocity;
    crrcSlider*        slider_windDir;
    crrcSlider*        slider_windTurbulence;
   
    crrcSlider*        slider_thermalStrengthMean;
    crrcSlider*        slider_thermalStrengthSigma;
    crrcSlider*        slider_thermalRadiusMean;
    crrcSlider*        slider_thermalRadiusSigma;
    crrcSlider*        slider_thermalLifetimeMean;
    crrcSlider*        slider_thermalLifetimeSigma;
    crrcSlider*        slider_thermalDensity;
   
    puInput*           inputNewWind;
    puInput*           inputNewThermal;
  
    // Thermal v3 data is not adjustable in GUI, but it has to be handled
    SimpleXMLTransfer* thermalv3data;
};

#endif // CRRC_WINDTHERMAL_H
