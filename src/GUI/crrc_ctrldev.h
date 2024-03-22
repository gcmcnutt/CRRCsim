/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2008 Jan Reucker (original author)
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
  

#ifndef CRRC_CTRLDEVICE_H
#define CRRC_CTRLDEVICE_H

#include <plib/pu.h>
#include <plib/puAux.h>
#include <string>

#include "crrc_dialog.h"

class CGUICtrlDeviceDialog : public CRRCDialog
{
  public:
    CGUICtrlDeviceDialog(CRRCDialog* parent);
    ~CGUICtrlDeviceDialog();
  
    const char* getDeviceName();
    void        setDeviceComboBoxByName(std::string sName);
    void        setSpeedComboBoxByName(std::string sName, char **list);
    bool        activateSelectedInterface();
    void        rebuildDeviceComboList();

    puaComboBox*  combo_inputMethod;
    puaComboBox*  combo_device;
    char**        combo_device_list;
    int           nCombo_device;
  
    puaComboBox*  combo_speed;

};

#endif  // CRRC_CTRLDEVICE_H
