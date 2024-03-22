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
  

#include "../i18n.h"
#include "crrc_ctrldev.h"
#include "crrc_ctrlgen.h"

#include "../global.h"
#include "../defines.h"
#include <crrc_config.h>
#include "crrc_gui_main.h"
#include "../crrc_main.h"
#include "util.h"
#include "../config.h"

#include "../mod_inputdev/inputdev_serial/inputdev_serial.h"
#include "../mod_inputdev/inputdev_software/inputdev_software.h"
#include "../mod_inputdev/inputdev_audio/inputdev_audio.h"

#define BUTTON_BOX_HEIGHT (2*DLG_DEF_SPACE+DLG_DEF_BUTTON_HEIGHT)
#define COMBO_W           (150)
#define COMBO_H           (DLG_DEF_BUTTON_HEIGHT)
#define LABEL_W           (180)
#define CHILD_OFFSET_X    (20)
#define CHILD_OFFSET_Y    (20)

static void CGUICtrlDeviceCallback(puObject *obj);
static void CGUICtrlDeviceDlgInputMethodChanged(puObject *obj);

static const char *serialSpeed[] = {"9600", "19200", NULL};


/**
 * Create the dialog as a child of another dialog
 *
 * \param parent            This dialog's parent dialog
 * \param ppcSpeedStrings   NULL-terminated list of speed value strings
 */
CGUICtrlDeviceDialog::CGUICtrlDeviceDialog(CRRCDialog* parent)
: CRRCDialog(0, 0, CRRC_DIALOG_OK)
{
  combo_inputMethod = new puaComboBox(LABEL_W + DLG_DEF_SPACE, 
                                      BUTTON_BOX_HEIGHT + 2*DLG_DEF_SPACE + 2*COMBO_H,
                                      LABEL_W + COMBO_W,
                                      BUTTON_BOX_HEIGHT + 2*DLG_DEF_SPACE + 3*COMBO_H,
                                      NULL, false);
  combo_inputMethod->setChildColourScheme(PUCLASS_POPUPMENU, dlgCol1[0], dlgCol1[1], dlgCol1[2]);
  combo_inputMethod->newList(Global::inputDev->InputMethodStringsGUI);
  combo_inputMethod->setLabelPlace(PUPLACE_LOWER_LEFT);
  combo_inputMethod->setLabel(_("Input Method"));
  combo_inputMethod->setCurrentItem(Global::TXInterface->inputMethod());
  combo_inputMethod->setCallback(CGUICtrlDeviceDlgInputMethodChanged);
  combo_inputMethod->setUserData(this);
  combo_inputMethod->reveal();
  combo_inputMethod->activate();
  
  // This widget must be constructed after the inputMethod combo box, because
  // it relies on combo_inputMethod pointing to the currently selected interface.
  combo_device_list = NULL;
  nCombo_device = 0;
  rebuildDeviceComboList();
  combo_device = new puaComboBox( LABEL_W + DLG_DEF_SPACE,
                                 BUTTON_BOX_HEIGHT + 1*DLG_DEF_SPACE + 1*COMBO_H,
                                 LABEL_W + COMBO_W,
                                 BUTTON_BOX_HEIGHT + 1*DLG_DEF_SPACE + 2*COMBO_H,
                                 NULL, false);
  combo_device->setChildColourScheme(PUCLASS_POPUPMENU, dlgCol1[0], dlgCol1[1], dlgCol1[2]);
  combo_device->newList(combo_device_list);
  combo_device->setLabelPlace(PUPLACE_LOWER_LEFT);
  combo_device->setLabel(_("Device"));
  combo_device->setUserData(this);
  combo_device->hide();
  setDeviceComboBoxByName(Global::TXInterface->getDeviceName());

  combo_speed = new puaComboBox( LABEL_W + DLG_DEF_SPACE,
                                BUTTON_BOX_HEIGHT + 0*DLG_DEF_SPACE + 0*COMBO_H,
                                LABEL_W + COMBO_W,
                                BUTTON_BOX_HEIGHT + 0*DLG_DEF_SPACE + 1*COMBO_H,
                                NULL, false);
  combo_speed->setChildColourScheme(PUCLASS_POPUPMENU, dlgCol1[0], dlgCol1[1], dlgCol1[2]);
  combo_speed->newList((char**)T_TX_InterfaceSerial::baudRates);
  combo_speed->setLabelPlace(PUPLACE_LOWER_LEFT);
  combo_speed->setLabel(_("Baudrate / Sync"));
  combo_speed->setCurrentItem(Global::TXInterface->getDeviceSpeed());
  combo_speed->setUserData(this);
  combo_speed->hide();

  // finalize the dialog
  close();
  setUserData(parent);
  setSize(COMBO_W + LABEL_W + 2*DLG_DEF_SPACE, 
          3*COMBO_H + 4*DLG_DEF_SPACE + BUTTON_BOX_HEIGHT);
  setCallback(CGUICtrlDeviceCallback);
  
  // center the dialog on screen: doesn't work well, as the combo box 
  // when expanded upwards, seems to influence getABox -- but also getBBox,
  // so using that doesn't help.
  // This is a child dialog, so let's place it slightly off-center
  int wwidth, wheight;
  int current_width = getABox()->max[0] - getABox()->min[0];
  int current_height = getABox()->max[1] - getABox()->min[1];
  puGetWindowSize(&wwidth, &wheight);
  setPosition(CHILD_OFFSET_X + wwidth/2 - current_width/2,
              CHILD_OFFSET_Y + wheight/2 - current_height/2);

  reveal();
  
  CGUICtrlDeviceDlgInputMethodChanged((puObject*)combo_inputMethod);
}

/**
 * Destroy the dialog
 */
CGUICtrlDeviceDialog::~CGUICtrlDeviceDialog()
{
  if (combo_device_list != NULL)
  {
    T_GUI_Util::freenames(combo_device_list, nCombo_device);
    combo_device_list = NULL;
    nCombo_device = 0;
  }
}


/**
 * Rebuild the list behind the device selection combo box
 *
 * The list is constructed, but *NOT* activated (by calling newList () on the
 * combo box widget). This must be done by the caller, because this method may
 * be called for the first time before the combo box is constructed.
 */
void CGUICtrlDeviceDialog::rebuildDeviceComboList()
{
  std::vector<std::string> Devices;
  int nItem = combo_inputMethod->getCurrentItem();

  if (nItem == T_TX_Interface::eIM_joystick)
  {
    T_TX_InterfaceSoftware::getDeviceList(Devices);
  }
  else if ((nItem == T_TX_Interface::eIM_zhenhua) 
            || (nItem == T_TX_Interface::eIM_serpic)
            || (nItem == T_TX_Interface::eIM_CT6A))
  {
    T_TX_InterfaceSerial::getDeviceList(Devices);
  }
  else if (nItem == T_TX_Interface::eIM_audio)
  {
    T_TX_InterfaceAudio::getDeviceList(Devices);
  }
  
  // clear old list (if it already exists)
  if (combo_device_list != NULL)
  {
    T_GUI_Util::freenames(combo_device_list, nCombo_device);
    combo_device_list = NULL;
    nCombo_device = 0;
  }
  combo_device_list = T_GUI_Util::loadnames(Devices, nCombo_device);
}


/**
 * Retrieve the item at index unDevice of the device combo box.
 */
const char* CGUICtrlDeviceDialog::getDeviceName()
{
  const char *name = "";
  if (combo_device_list != NULL)
  {
  int n=combo_device->getCurrentItem();
  if( n >= 0)
    {
    name = combo_device_list[n];
    }
  }
  return name;
}

/**
 * Set the current item of the device combo box by name
 */
void CGUICtrlDeviceDialog::setDeviceComboBoxByName(std::string sName)
{
  int nIndex = T_GUI_Util::findname(combo_device_list, sName);
  if (nIndex < 0)
  {
    nIndex = 0;
  }
  combo_device->setCurrentItem(nIndex);
}


/**
 * Set the current item of the speed combo box by name
 */
void CGUICtrlDeviceDialog::setSpeedComboBoxByName(std::string sName, char** list)
{
  int nIndex = T_GUI_Util::findname(list, sName);
  if (nIndex < 0)
  {
    nIndex = 0;
  }
  combo_speed->setCurrentItem(nIndex);
}


/** \brief Activate the selected interface
 *
 *  Try to activate the interface selected by combo_inputMethod and
 *  the device settings.
 *
 *  \retval true  selected interface has been activated successfully
 *  \retval false unable to activate selected interface, fell back to mouse
 */
bool CGUICtrlDeviceDialog::activateSelectedInterface()
{
  bool activateOK = true;
  int  nItem      = combo_inputMethod->getCurrentItem();
  
  cfgfile->setAttributeOverwrite("inputMethod.method", Global::inputDev->InputMethodStrings[nItem]);

  // directly accessing the config file is a dirty hack, but I don't know
  // a better way to handle this situation
  if (nItem == T_TX_Interface::eIM_serpic)
  {
    cfgfile->setAttributeOverwrite("inputMethod.FMSPIC.port.name", getDeviceName());
    cfgfile->setAttributeOverwrite("inputMethod.FMSPIC.port.baudrate", combo_speed->getStringValue());
    if (combo_speed->getIntegerValue() == 9600)
    {
      cfgfile->setAttributeOverwrite("inputMethod.FMSPIC.port.sync", "240"); // sync byte 0xF0
      cfgfile->setAttributeOverwrite("inputMethod.FMSPIC.port.button_channel", "1"); // has button byte
    }
    else
    {
      cfgfile->setAttributeOverwrite("inputMethod.FMSPIC.port.sync", "255"); // sync byte 0xFF
      cfgfile->setAttributeOverwrite("inputMethod.FMSPIC.port.button_channel", "0"); // no button byte
    }
  }
  else if (nItem == T_TX_Interface::eIM_zhenhua)
  {
    cfgfile->setAttributeOverwrite("inputMethod.zhenhua.port.name", getDeviceName());
    cfgfile->setAttributeOverwrite("inputMethod.zhenhua.port.baudrate", combo_speed->getStringValue());
  }
  else if (nItem == T_TX_Interface::eIM_CT6A)
  {
    cfgfile->setAttributeOverwrite("inputMethod.CT6A.port.name", getDeviceName());
    cfgfile->setAttributeOverwrite("inputMethod.CT6A.port.baudrate", combo_speed->getStringValue());
  }
  else if (nItem == T_TX_Interface::eIM_joystick)
  {
    int n = combo_device->getCurrentItem();
    cfgfile->setAttributeOverwrite("inputMethod.joystick.number", n);
    if (Global::inputDev->joystick_n != n)
    {
      Global::inputDev->openJoystick(n);
    }
  }
  else if (nItem == T_TX_Interface::eIM_audio)
  {
    cfgfile->setAttributeOverwrite("inputMethod.audio.device_name", getDeviceName());
  }
  
  std::string msg = reconfigureInputMethod();
  if (msg.length())
  {
    Global::gui->errorMsg(msg.c_str());
    activateOK = false;
    combo_inputMethod->setCurrentItem(Global::TXInterface->inputMethod());
  }
  CGUICtrlDeviceDlgInputMethodChanged((puObject*)combo_inputMethod);

  return activateOK;
}


/** \brief The dialog's callback.
 *
 * The callback tries to initialize the new interface. If initialization
 * fails, don't accept an "OK".
 */
void CGUICtrlDeviceCallback(puObject *obj)
{
  CGUICtrlDeviceDialog* dlg = static_cast<CGUICtrlDeviceDialog*>(obj);
  bool boDoExit = true;
  
  if (obj->getIntegerValue() == CRRC_DIALOG_OK)
  {    
    // Dialog left by clicking OK
    
    // if inputMethod has changed:
    if (!dlg->activateSelectedInterface())
    {
      // interface settings are somehow wrong, don't quit the dialog
      boDoExit = false;
    }
  }
  
  if (boDoExit)
  {
    // propagate settings to parent dialog
    CGUICtrlGeneralDialog* parent = static_cast<CGUICtrlGeneralDialog*>(dlg->getUserData());
    parent->updateInputMethod();
    parent->showHideButtons();
    
    // close dialog
    puDeleteObject(obj);
  }
}


void CGUICtrlDeviceDlgInputMethodChanged(puObject *obj)
{
  CGUICtrlDeviceDialog* dlg = static_cast<CGUICtrlDeviceDialog*>(obj->getUserData());
  int nItem = dlg->combo_inputMethod->getCurrentItem();

  // show "Device..." button?
  if (nItem == T_TX_Interface::eIM_serpic)
  {
    dlg->rebuildDeviceComboList();
    dlg->combo_device->newList(dlg->combo_device_list);
    dlg->combo_device->reveal();
    
    dlg->combo_speed->reveal();
    dlg->combo_speed->newList((char**)serialSpeed);
    // How do we get the currently configured settings? Looking into the config
    // file seems to be some kind of hack and breaks OO rules, but we can't
    // ask the interface object itself, because it may not be constructed yet.
    // And the global TXInterface may still be of a different type.
    std::string port = cfgfile->getString("inputMethod.FMSPIC.port.name", "");
    std::string baudrate = cfgfile->getString("inputMethod.FMSPIC.port.baudrate", "19200");
    dlg->setDeviceComboBoxByName(port);
    dlg->setSpeedComboBoxByName(baudrate, (char**)serialSpeed);
  }
  else if (nItem == T_TX_Interface::eIM_zhenhua)
  {
    dlg->rebuildDeviceComboList();
    dlg->combo_device->newList(dlg->combo_device_list);
    dlg->combo_device->reveal();

    dlg->combo_speed->reveal();
    dlg->combo_speed->newList((char**)T_TX_InterfaceSerial::baudRates);
    // How do we get the currently configured settings? Looking into the config
    // file seems to be some kind of hack and breaks OO rules, but we can't
    // ask the interface object itself, because it may not be constructed yet.
    // And the global TXInterface may still be of a different type.   
    std::string port = cfgfile->getString("inputMethod.zhenhua.port.name", "");
    std::string baudrate = cfgfile->getString("inputMethod.zhenhua.port.baudrate", "19200");
    dlg->setDeviceComboBoxByName(port);
    dlg->setSpeedComboBoxByName(baudrate, (char**)T_TX_InterfaceSerial::baudRates);
  }
  else if (nItem == T_TX_Interface::eIM_CT6A)
  {
    dlg->rebuildDeviceComboList();
    dlg->combo_device->newList(dlg->combo_device_list);
    dlg->combo_device->reveal();

    dlg->combo_speed->reveal();
    dlg->combo_speed->newList((char**)T_TX_InterfaceSerial::baudRates);
    // How do we get the currently configured settings? Looking into the config
    // file seems to be some kind of hack and breaks OO rules, but we can't
    // ask the interface object itself, because it may not be constructed yet.
    // And the global TXInterface may still be of a different type.   
    std::string port = cfgfile->getString("inputMethod.CT6A.port.name", "");
    std::string baudrate = cfgfile->getString("inputMethod.CT6A.port.baudrate", "115200");
    dlg->setDeviceComboBoxByName(port);
    dlg->setSpeedComboBoxByName(baudrate, (char**)T_TX_InterfaceSerial::baudRates);
  }
  else if (nItem == T_TX_Interface::eIM_joystick)
  {
    dlg->rebuildDeviceComboList();
    dlg->combo_device->newList(dlg->combo_device_list);
    dlg->combo_device->reveal();
    // How do we get the currently configured settings? Looking into the config
    // file seems to be some kind of hack and breaks OO rules, but we can't
    // ask the interface object itself, because it may not be constructed yet.
    // And the global TXInterface may still be of a different type.
    int n = cfgfile->attributeAsInt("inputMethod.joystick.number", 0);
    dlg->combo_device->setCurrentItem(n);

    dlg->combo_speed->hide();
  }
  else if (nItem == T_TX_Interface::eIM_audio)
  {
    dlg->rebuildDeviceComboList();
    dlg->combo_device->newList(dlg->combo_device_list);
    dlg->combo_device->reveal();
    // How do we get the currently configured settings? Looking into the config
    // file seems to be some kind of hack and breaks OO rules, but we can't
    // ask the interface object itself, because it may not be constructed yet.
    // And the global TXInterface may still be of a different type.
    std::string sounddev = cfgfile->getString("inputMethod.audio.device_name", "default");
    dlg->setDeviceComboBoxByName(sounddev);

    dlg->combo_speed->hide();
  }
  else
  {
    dlg->combo_device->hide();
    dlg->combo_speed->hide();
  }
}


