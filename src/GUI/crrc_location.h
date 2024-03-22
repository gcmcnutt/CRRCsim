/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2008, 2010 Jan Reucker (original author)
 * Copyright (C) 2008 Jens Wilhelm Wulf
 * Copyright (C) 2008, 2012 Joel Lienard
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
  

// crrc_location.h - Location selection dialog

#ifndef CRRC_LOCATION_H
#define CRRC_LOCATION_H

#include <plib/pu.h>
#include <plib/puAux.h>
#include <vector>
#include <string>

#include "crrc_dialog.h"
#include "puaScrListBox.h"
#include "puaGLPreview.h"
#include "puaImageFrame.h"


class CGUILocationDialog;

/** \brief The location selection dialog.
 *
 *  This dialog provides a list box with all
 *  available locations.
 */
class CGUILocationDialog : public CRRCDialog
{
  public:
    CGUILocationDialog();
    ~CGUILocationDialog();
  
    std::string getLocation() const;
    int getLocationId() const;
 
    /// Save selected location
    bool  saveSelection() const;

    /// Update location info
    void updateLocationInfo(int);
      
  private:
    puaImageFrame  *preview;                 /// preview of the selected location
    puaScrListBox  *cbox;
    puaComboBox    *sbox;
    puText         *ptext;
    puaLargeInput  *description;
    std::string    path_string;
    std::string    description_string;
    
    std::vector<std::string> fileslist;
    std::vector<std::string> locationslist;

    char **locationsList; 
    char **filesList;
    char **skiesList;
    int filesListSize;
    int locationsListSize;
    int skiesListSize;

    ssgTexture *preview_texture;
    void lists_insert(std::string name, std::string fullpath);
    int index_in_locationslist(std::string lname);
    
    static void CGUISkySelectCallback(puObject *obj);
};

#endif // CRRC_LOCATION_H
