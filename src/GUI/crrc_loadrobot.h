/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 * 
 * Copyright (C) 2010 Jens Wilhelm Wulf (original author)
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
  


#ifndef CRRC_LOADROBOT_H
#define CRRC_LOADROBOT_H

#include <plib/pu.h>
#include <plib/puAux.h>
#include <vector>

#include "crrc_dialog.h"
#include "puaFileBox.h"


class CGUILoadRobotDialog;

/** \brief robot/flight log load dialog
 *
 */
class CGUILoadRobotDialog : public CRRCDialog
{
public:
  CGUILoadRobotDialog();
  ~CGUILoadRobotDialog();
  
  /**
   *  Update the dialog when a new file is selected.
   */
  void  updateFileInfo();

  std::string getFilename();
  
  puaFileBox* files;
  puButton* check_demo;
  
private:
  
  puaLargeInput* description;
  std::string description_string;
};

#endif // CRRC_LOADROBOT_H
