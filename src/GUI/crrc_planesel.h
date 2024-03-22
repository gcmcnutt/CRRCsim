/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project

 * Copyright (C) 2005, 2006, 2007, 2008 Jan Reucker (original author)
 * Copyright (C) 2007 Tom Willis
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
  

// crrc_planesel.h - Airplane selection dialog

#ifndef CRRC_PLANESEL_H
#define CRRC_PLANESEL_H

#include <plib/pu.h>
#include <plib/puAux.h>
#include <vector>

#include "crrc_dialog.h"
#include "puaScrListBox.h"
#include "puaGLPreview.h"
#include "../mod_misc/SimpleXMLTransfer.h"


class CGUIPlaneSelectDialog;

/** \brief The airplane selection dialog.
 *
 *  This dialog provides a scrollable list with all
 *  installed aircraft.
 */
class CGUIPlaneSelectDialog : public CRRCDialog
{
  public:
    CGUIPlaneSelectDialog();
    ~CGUIPlaneSelectDialog();
  
    /// Update the dialog depending on the currently selected file
    void updateFileInfo();
    
    /// Assign a list of planes in the current category to planeList
    void updatePlaneList();
  
    /// Update the OpenGL preview based on a UI change
    void updatePreview();

    /// Save selected file and options to config file
    bool saveSelection() const;
  
    /// get value of the "load launch default" checkbox
    int getLoadLaunchDefault() const {return check_useLaunchDefault->getIntegerValue();};
  
    /// get value of the "load mixer default" checkbox
    int getLoadMixerDefault() const {return check_useMixerDefault->getIntegerValue();};
  
  private:
    puaComboBox    *cat;
    char          **catList;
    int           catListSize;

    puaScrListBox *planes;
    char          **planeList;              ///< names of the planes
    int           planeListSize;
    std::vector<std::string> plane_paths;   ///< full paths of model files, same order as planeList

    puaGLPreview  *preview;                 ///< GL preview of the selected plane

    puaComboBox    *gbox;
    char**        optsGraphics;
    int           nOptsGraphics;
    SimpleXMLTransfer* optsGrpGraphics;

    puaComboBox    *cbox;
    char**        optsConfig;
    int           nOptsConfig;
    SimpleXMLTransfer* optsGrpConfig;
  
    puText        *location_label;
    std::string   location_label_string;
  
    puaLargeInput  *description;
    std::string   description_string;
    
    puButton      *check_useLaunchDefault;
  
    puButton      *check_useMixerDefault;
  
    /// Fill the mfiles vector with the paths of all model files
    void  createMFileList(std::vector<std::string>& filelist);
    
    /// Assign a list of categories to the catList
    void  updateCategories();
    
    /// Get all planes of one category
    void  getPlanesByCategory(std::vector<std::string>& planes, std::string category);
    
    /// Get the name of a model
    std::string getModelName(std::string path);
    
    /// Clean up config boxes
    void cleanUpConfigAndGraphics();
};

#endif // CRRC_PLANESEL_H
