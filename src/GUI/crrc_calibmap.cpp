/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2008, 2009, 2010 Jan Reucker (original author)
 * Copyright (C) 2007 Chris Bayley
 * Copyright (C) 2008, 2010 Jens Wilhelm Wulf
 * Copyright (C) 2008 Olivier Bordes
 * Copyright (C) 2012 Joel Lienard
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
#include "../i18n.h"
#include <iostream>

#include "../global.h"
#include "crrc_calibmap.h"
#include "../crrc_main.h"

#define BUTTON_BOX_HEIGHT   (3*DLG_DEF_SPACE+2*DLG_DEF_BUTTON_HEIGHT)
#define INPUT_H       (25)
#define INPUT_W       (40)
#define LABEL_W       (100)
#define LABEL_H       (20)
#define SLIDER_W      (17)
#define SLIDER_SZ     (100)
#define SLIDER_SP     (8)
#define HORIZ_SPACER  (35)
#define COMBO_W       (120)

static const char *mapped_to_keyboard = "K";

static char *radioTypes[] = {(char*)RadioTypeStrings[0], (char*)RadioTypeStrings[1],
                             (char*)RadioTypeStrings[2], (char*)RadioTypeStrings[3],
                             (char*)RadioTypeStrings[4], (char*)RadioTypeStrings[5],
                             (char*)RadioTypeStrings[6],
                             NULL};

static void CGUICalibMapCallback(puObject *obj);
static void CGUICalibButtonCallback(puObject *obj);
static void CGUIComboCallback(puObject *obj);
static void CGUIMapCallback(puObject *obj);

CGUICalibMapDialog::CGUICalibMapDialog()
  : CRRCDialog(0, 0, CRRC_DIALOG_OK), state(0), numaxis(0)
{
  // activate the test mode to see the movements of the model
  activate_test_mode();
  
  //activate verbosity
  sVerbosity = Global::nVerbosity;
  Global::nVerbosity  =1;
  
  // create some label strings from scratch
  // ("0", "1", ...)
  for (int i = 0; i < TX_MAXAXIS; i++)
  {
    // attention: only works for TX_MAXAXIS < 10
    axislabel[i] = new char[2];
    *(axislabel[i]) = 0x30 + i;
    *(axislabel[i] + 1) = '\0';
  }
  axislabel[TX_MAXAXIS] = NULL;
  
  // Create the selection list for the SelectBoxes.
  // Contains as many axis as the interface features,
  // plus "-" (deactivate) and NULL (terminate).
  // The number of axis will be stored to make sure
  // it doesn't change until the dialog is closed.
  numaxis = Global::TXInterface->getNumAxes();
  
  if (numaxis > TX_MAXAXIS)
  {
    numaxis = TX_MAXAXIS;
  }
  for (int i = 0; i < numaxis; i++)
  {
    selectlist[i] = axislabel[i];
  }
  selectlist[numaxis]   = mapped_to_keyboard;
  selectlist[numaxis+1] = NULL;

  // The "mapping group" (left part of the dialog).
  // The combo box on top is not part of the group
  // because it controls hiding and revealing of the
  // other widgets.
  mapgroup = new puGroup(LABEL_W + DLG_DEF_SPACE,
                         BUTTON_BOX_HEIGHT);
  for (int i = 0; i < NUM_FUNCTIONS; i++)
  {
    int index = NUM_FUNCTIONS - 1;
    axismapping[i] = new puaSelectBox(0,
                                     (index-i)* DLG_DEF_SPACE + (index-i) * INPUT_H,
                                     INPUT_W,
                                     (index-i)* DLG_DEF_SPACE + (index+1-i) * INPUT_H,
                                     (char **)selectlist
                                    );
    axismapping[i]->setLabelPlace(PUPLACE_CENTERED_LEFT);
    axismapping[i]->setLabel(Global::inputDev->AxisStringsGUI[i+1]);
    int item = Global::TXInterface->map->func[i+1];
    if (item < 0)
    {
      // disable (set to "K")
      item = numaxis;
    }
    axismapping[i]->setCurrentItem(item);
    axismapping[i]->setUserData(this);
    axismapping[i]->setCallback(CGUIMapCallback);

    invert[i] = new puButton(DLG_DEF_SPACE + INPUT_W,
                             4 + (index-i)* DLG_DEF_SPACE + (index-i) * INPUT_H,
                             DLG_DEF_SPACE + INPUT_W + DLG_CHECK_W,
                             4 + (index-i)* DLG_DEF_SPACE + (index-i) * INPUT_H + DLG_CHECK_H);
    invert[i]->setButtonType(PUBUTTON_VCHECK);
    invert[i]->setLabelPlace(PUPLACE_CENTERED_RIGHT);
    invert[i]->setLabel(_("Inv"));
    if (Global::TXInterface->map->inv[i+1] > 0)
    {
      invert[i]->setValue(0);
    }
    else
    {
      invert[i]->setValue(1);
    }
    invert[i]->setUserData(this);
    invert[i]->setCallback(CGUIMapCallback);
  }
  mapgroup->close();
  mapgroup->hide();

  // The combo box above the "mapping group"
  combo_radiotype = new puaComboBox(LABEL_W + DLG_DEF_SPACE,
                                   BUTTON_BOX_HEIGHT + (NUM_FUNCTIONS)* DLG_DEF_SPACE + (NUM_FUNCTIONS) * INPUT_H,
                                   LABEL_W + DLG_DEF_SPACE + COMBO_W,
                                   BUTTON_BOX_HEIGHT + (NUM_FUNCTIONS)* DLG_DEF_SPACE + (NUM_FUNCTIONS + 1) * INPUT_H,
                                   NULL, false);
  combo_radiotype->setChildColourScheme(PUCLASS_POPUPMENU, dlgCol1[0], dlgCol1[1], dlgCol1[2]);
  combo_radiotype->newList(radioTypes);
  combo_radiotype->setLabelPlace(PUPLACE_CENTERED_LEFT);
  combo_radiotype->setLabel(_("Radio type"));
  combo_radiotype->setUserData(this);
  combo_radiotype->setCurrentItem(Global::TXInterface->map->radioType());
  combo_radiotype->setCallback(CGUIComboCallback);

  if (combo_radiotype->getCurrentItem() == T_AxisMapper::CUSTOM)
  {
    mapgroup->reveal();
  }

  if ((Global::TXInterface->inputMethod() == T_TX_Interface::eIM_mouse)
        ||
      (Global::TXInterface->inputMethod() == T_TX_Interface::eIM_joystick))
  {
    Global::TXInterface->map->setRadioType(T_AxisMapper::CUSTOM);
    combo_radiotype->setCurrentItem(Global::TXInterface->map->radioType());
    combo_radiotype->hide();
  }
  else
  {
    combo_radiotype->reveal();
  }
  
  // The "calibration group" (right part of the dialog).
  calibgroup = new puGroup(LABEL_W + 3*DLG_DEF_SPACE + DLG_CHECK_W + INPUT_W + HORIZ_SPACER,
                           BUTTON_BOX_HEIGHT + DLG_DEF_SPACE);
  
  for (int i = 0; i < TX_MAXAXIS; i++)
  {
    axis[i] = new puSlider( i*SLIDER_SP + i * SLIDER_W,
                            LABEL_H + DLG_DEF_SPACE + DLG_DEF_BUTTON_HEIGHT,
                            SLIDER_SZ,
                            TRUE,
                            SLIDER_W);
    axis[i]->setLabelPlace(PUPLACE_BOTTOM_CENTERED);
    axis[i]->setLabel(axislabel[i]);
    axis[i]->setValue((float)0.5);
    
    // set min and max to some excessive values
    minval[i] = 1.0e30;
    maxval[i] = -1.0e30;
  }
  
  butCalibrate = new puOneShot( 0,
                                0,
                                DLG_DEF_BUTTON_WIDTH + 10,
                                DLG_DEF_BUTTON_HEIGHT);
  butCalibrate->setLegend(_("Calibrate"));
  butCalibrate->setLabelPlace(PUPLACE_BOTTOM_LEFT);
  butCalibrate->setLabel("");
  butCalibrate->setUserData(this);
  butCalibrate->setCallback(CGUICalibButtonCallback);
  
  if (Global::TXInterface->usesCalibration())
  {
    butCalibrate->reveal();
  }
  else
  {
    butCalibrate->hide();
  }

  calibgroup->close();

  // finalize the dialog
  close();
  setSize(LABEL_W + (TX_MAXAXIS -1)*SLIDER_SP + DLG_DEF_SPACE+ DLG_CHECK_W + INPUT_W + 2*HORIZ_SPACER + TX_MAXAXIS * SLIDER_W,
          BUTTON_BOX_HEIGHT + (NUM_FUNCTIONS+3) * DLG_DEF_SPACE + (NUM_FUNCTIONS+1) * INPUT_H);  
  setCallback(CGUICalibMapCallback);
  centerOnScreen();
  
  //set the dialogue transparent and hide the others dialogues to better see the model
  setTransparency(.3);
  hideOthers();
  
  reveal();
}


CGUICalibMapDialog::~CGUICalibMapDialog()
{
  // clean up
  for (int i = 0; i < NUM_FUNCTIONS; i++)
  {
    puDeleteObject(axismapping[i]);
    puDeleteObject(invert[i]);
  }
  for (int i = 0; i < TX_MAXAXIS; i++)
  {
    puDeleteObject(axis[i]);
    delete[] axislabel[i];
  }
  puDeleteObject(butCalibrate);
  
  revealAll();//set visible all dialogues previously hidden 
  leave_test_mode();
  
  //restore verbosity
  Global::nVerbosity = sVerbosity;
}


void CGUICalibMapDialog::update()
{
  float afValues[TX_MAXAXIS];
  
  Global::TXInterface->getRawData(afValues);
  
  for (int i = 0; i < numaxis; i++)
  {
    float f = afValues[i];
    if (f < minval[i])
    {
      minval[i] = f;
    }
    if (f > maxval[i])
    {
      maxval[i] = f;
    }
    
    if (Global::TXInterface->usesCalibration())
    {
      switch (state)
      {
        case 1:
          // Track min and max values
          if (maxval[i] != minval[i])
          {
            Global::TXInterface->calib->setValMinMax(i, minval[i], maxval[i]);
          }
          else
          {
            Global::TXInterface->calib->setValMinMax(i, f - 1, f + 1);
          }
          Global::TXInterface->calib->setValMid(i, (maxval[i] + minval[i]) * 0.5);
          break;
        
        default:
          break;
      }

      float value = static_cast<float>(0.5 + Global::TXInterface->calib->calibrate(i, f));
      axis[i]->setValue(value);
    }
    else
    {
      // no calibration, just display raw values
      axis[i]->setValue(static_cast<float>(0.5 + f));
    }
  }
}

void CGUICalibMapDialog::save_mapping ()
{
  // save mapping
  int rtype = combo_radiotype->getCurrentItem();
  if (rtype == T_AxisMapper::CUSTOM)
  {
    for (int i = 1; i <= NUM_FUNCTIONS; i++)
    {
      int axis = axismapping[(i-1)]->getCurrentItem();
      if (axis < numaxis)
      {
        // function is mapped to an axis
        Global::TXInterface->map->func[i] = axis;
      }
      else
      {
        // unmapped, disable function
        Global::TXInterface->map->func[i] = -1;
      }
      if (invert[(i-1)]->getIntegerValue() == 1)
      {
        Global::TXInterface->map->inv[i] = -1.0;
      }
      else
      {
        Global::TXInterface->map->inv[i] = 1.0;
      }
    }
    Global::TXInterface->map->saveToCustom();
  }
  Global::TXInterface->map->setRadioType(rtype);
}

/**************************/
static void CGUICalibMapCallback(puObject *obj)
{
  CGUICalibMapDialog* dlg = static_cast<CGUICalibMapDialog*>(obj);
  // only react to buttons if calibration is not running
  if (dlg->state == 0)
  {
    if (dlg->getIntegerValue() == CRRC_DIALOG_OK)
    {
      dlg->save_mapping();
    }
    else
    {
    }
    puDeleteObject(dlg);
  }
}

static void CGUICalibButtonCallback(puObject *obj)
{
  CGUICalibMapDialog* dlg = static_cast<CGUICalibMapDialog*>(obj->getUserData());
  
  int state = dlg->state;  
  state++;
  if (state > 2)
  {
    state = 0;

    std::cout << "Calibration:\n";
    float afValues[TX_MAXAXIS];  
    Global::TXInterface->getRawData(afValues);    
    for (int i = 0; i < dlg->numaxis; i++)
    {
      Global::TXInterface->calib->setValMid(i, afValues[i]);
      Global::TXInterface->calib->PrintSettings(i);
    }
  }
  
  switch (state)
  {
    default:
    case 0:
      dlg->butCalibrate->setLegend(_("Calibrate"));
      dlg->butCalibrate->setLabel("");
      dlg->unlock();
      break;
    
    case 1:
      dlg->butCalibrate->setLegend(_("Next"));
      dlg->butCalibrate->setLabel(_("Move controls to all extents!"));
      dlg->lock();
      break;
    
    case 2:
      dlg->butCalibrate->setLegend(_("Finish"));
      dlg->butCalibrate->setLabel(_("Now center all controls!"));
      break;
  }
  
  dlg->state = state;
}

static void CGUIComboCallback(puObject *obj)
{
  CGUICalibMapDialog* dlg = static_cast<CGUICalibMapDialog*>(obj->getUserData());
  puaComboBox* combo = static_cast<puaComboBox*>(obj);
  
  Global::TXInterface->map->setRadioType(combo->getCurrentItem());
  
    // show or hide the mapping group
  if (combo->getCurrentItem() == T_AxisMapper::CUSTOM)
  {
    dlg->mapgroup->reveal();

    for (int i = 0; i < NUM_FUNCTIONS; i++)
    {
      dlg->axismapping[i]->setCurrentItem(Global::TXInterface->map->func[i+1]);
      if (Global::TXInterface->map->inv[i+1] > 0)
      {
        dlg->invert[i]->setValue(0);
      }
      else
      {
        dlg->invert[i]->setValue(1);
      }
    }
  }
  else
  {
    dlg->mapgroup->hide();
  }
}

static void CGUIMapCallback(puObject *obj)
{
  CGUICalibMapDialog* dlg = static_cast<CGUICalibMapDialog*>(obj->getUserData());
  dlg->save_mapping();
}

