/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2008 Jens Wilhelm Wulf (original author)
 * Copyright (C) 2005, 2006, 2007, 2008, 2009 Jan Reucker
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
  
#include "i18n.h"
#include "mouse_kbd.h"
#include "defines.h"
#include "GUI/util.h"
#include "mod_misc/lib_conversions.h"
#include "mod_fdm/fdm_inputs.h"

#include <stdio.h>
#include <sstream>

void TInputDev::init(SimpleXMLTransfer* cfgfile)
{
  int size;
  SimpleXMLTransfer* item;
  SimpleXMLTransfer* group;
  SimpleXMLTransfer* item2;

  // set defaults
  for (int n=0; n<MAXJOYBUTTON + 1; n++)
    joystick_bind_b[n] = NOTHING;

  item = cfgfile->getChild("inputMethod.mouse.bindings.buttons", true);

  mouse_bind_l= getValButton(item->attribute("l",    "RESUME"), RESUME);
  mouse_bind_m= getValButton(item->attribute("m",    "RESET"),  RESET);
  mouse_bind_r= getValButton(item->attribute("r",    "PAUSE"),  PAUSE);
  mouse_bind_u= getValButton(item->attribute("up",   "INCTHROTTLE"),  INCTHROTTLE);
  mouse_bind_d= getValButton(item->attribute("down", "DECTHROTTLE"),  DECTHROTTLE);

  item = cfgfile->getChild("inputMethod.joystick", true);
  joystick_n = item->attributeAsInt("number", 0);

  group = item->getChild("bindings.buttons", true);
  size  = group->getChildCount();
  if (size > MAXJOYBUTTON + 1)
    size = MAXJOYBUTTON + 1;
  for (int n=0; n<size; n++)
  {
    item2 = group->getChildAt(n);
    joystick_bind_b[n] = getValButton(item2->attribute("bind", "NOTHING"), NOTHING);
  }

  // zoom control
  std::string zctr = strU(cfgfile->getString("zoom.control", ZoomControlStrings[0]));
  for (int n=0; n<NUM_ZOMCONTROL; n++)
  {
    if (zctr.compare(ZoomControlStrings[n]) == 0)
            zoom_control = n;
  }

}

void TInputDev::putBackIntoCfg(SimpleXMLTransfer* cfgfile)
{
  int size;
  SimpleXMLTransfer* item; // = cfgfile->getChild("inputMethod.mouse.bindings.axes");
  SimpleXMLTransfer* group;
  SimpleXMLTransfer* item2;

  item = cfgfile->getChild("inputMethod.mouse.bindings.buttons");

  item->setAttributeOverwrite("l",    ActionButtonStrings[mouse_bind_l]);
  item->setAttributeOverwrite("m",    ActionButtonStrings[mouse_bind_m]);
  item->setAttributeOverwrite("r",    ActionButtonStrings[mouse_bind_r]);
  item->setAttributeOverwrite("up",   ActionButtonStrings[mouse_bind_u]);
  item->setAttributeOverwrite("down", ActionButtonStrings[mouse_bind_d]);

  item = cfgfile->getChild("inputMethod.joystick");
  item->setAttributeOverwrite("number", joystick_n);

  group = item->getChild("bindings.buttons");

  // clean list
  size = group->getChildCount();
  for (int n=0; n<size; n++)
  {
    item2 = group->getChildAt(0);
    group->removeChildAt(0);
    delete item2;
  }
  // create new list
  for (int n=0; n<MAXJOYBUTTON + 1; n++)
  {
    item2 = new SimpleXMLTransfer();
    item2->setName("button");
    item2->addAttribute("bind", ActionButtonStrings[joystick_bind_b[n]]);
    group->addChild(item2);
  }

  // zoom control
  cfgfile->setAttributeOverwrite("zoom.control",ZoomControlStrings[zoom_control]);

}

int  TInputDev::getValAxis  (std::string asString, int nDefault)
{
  asString = strU(asString);

  if (asString.compare("AILERON") == 0)
    return(T_AxisMapper::AILERON);
  else if (asString.compare("ELEVATOR") == 0)
    return(T_AxisMapper::ELEVATOR);
  else if (asString.compare("RUDDER") == 0)
    return(T_AxisMapper::RUDDER);
  else if (asString.compare("THROTTLE") == 0)
    return(T_AxisMapper::THROTTLE);
  else
    return(nDefault);
}

int  TInputDev::getValButton(std::string asString, int nDefault)
{
  asString = strU(asString);

  if (asString.compare("RESUME") == 0)
    return(RESUME);
  else if (asString.compare("PAUSE") == 0)
    return(PAUSE);
  else if (asString.compare("RESET") == 0)
    return(RESET);
  else if (asString.compare("INCTHROTTLE") == 0)
    return(INCTHROTTLE);
  else if (asString.compare("DECTHROTTLE") == 0)
    return(DECTHROTTLE);
  else if (asString.compare("ZOOMIN") == 0)
    return(ZOOMIN);
  else if (asString.compare("ZOOMOUT") == 0)
    return(ZOOMOUT);
  else
    return(nDefault);
}

// description: see header file
std::string TInputDev::openJoystick()
{
#if TEST_WITHOUT_JOYSTICK == 0
  int numJoysticks;
  
  // check if the joystick subsystem is initialized,
  // and maybe initialize again
  if (!SDL_WasInit(SDL_INIT_JOYSTICK))
  {
    printf("TInputDev::openJoystick: Joystick subsystem not initialized, doing it now...\n");
    SDL_InitSubSystem(SDL_INIT_JOYSTICK);
    if (!SDL_WasInit(SDL_INIT_JOYSTICK))
    {
      std::string SDLerr = SDL_GetError();
      printf("Can't initialize joystick subsystem!\n");
      printf("SDL says: %s\n", SDLerr.c_str());
      return ("Unable to initialize joystick subsystem. " + SDLerr);
    }
  }
  
  closeJoystick();
  
  numJoysticks = SDL_NumJoysticks();
  printf("TInputDev::openJoystick: SDL found %d joysticks\n", numJoysticks);
  if (numJoysticks > 0)
  {
    for (int i = 0; i < numJoysticks; i++)
    {
      printf("%d: %s\n", i, SDL_JoystickName(i));
    }
    printf("Trying to open joystick %d\n", joystick_n);
    joy = SDL_JoystickOpen(joystick_n);
    if (joy)
    {
      printf("Opened Joystick %d\n", joystick_n);
      printf("Name: %s\n", SDL_JoystickName(joystick_n));
      printf("Number of Axes: %d\n", SDL_JoystickNumAxes(joy));
      printf("Number of Buttons: %d\n", SDL_JoystickNumButtons(joy));
      return("");
    }
    else
    {
      std::string SDLerr = SDL_GetError();
      printf("SDL says: %s\n", SDLerr.c_str());
      return("Couldn't open Joystick " + itoStr(joystick_n, ' ', 1) + ": " + SDLerr);
    }
  }
  else
  {
    return(_("No Joysticks found\n"));
  }
#else
  printf("Opened fake joystick %d\n", joystick_n);
  return("");
#endif
}


// description: see header file
std::string TInputDev::openJoystick(int joy_n)
{
  joystick_n = joy_n;
  return openJoystick();
}


TInputDev::TInputDev()//constructor
{
  joy = (SDL_Joystick*)0;
  //Initialization of strings used for the configuration of the controls.
  {   //for XML
    std::vector<std::string> lAxisStringsXML(T_AxisMapper::NUM_AXISFUNCS);
    lAxisStringsXML[T_AxisMapper::NOTHING]  = "nothing";
    lAxisStringsXML[T_AxisMapper::AILERON]  = "aileron";
    lAxisStringsXML[T_AxisMapper::ELEVATOR] = "elevator";
    lAxisStringsXML[T_AxisMapper::RUDDER]   = "rudder";
    lAxisStringsXML[T_AxisMapper::THROTTLE] = "throttle";
    lAxisStringsXML[T_AxisMapper::FLAP]     = "flap";
    lAxisStringsXML[T_AxisMapper::SPOILER]  = "spoiler";
    lAxisStringsXML[T_AxisMapper::RETRACT]  = "retract";
    lAxisStringsXML[T_AxisMapper::PITCH]    = "pitch";
    int size;
    AxisStringsXML = T_GUI_Util::loadnames(lAxisStringsXML, size);
  }
  {   //for GUI, possibly translated      
    std::vector<std::string> lAxisStringsGUI(T_AxisMapper::NUM_AXISFUNCS);
    lAxisStringsGUI[T_AxisMapper::NOTHING]  =  _("nothing");
    lAxisStringsGUI[T_AxisMapper::AILERON]  =  _("Aileron");
    lAxisStringsGUI[T_AxisMapper::ELEVATOR] =  _("Elevator");
    lAxisStringsGUI[T_AxisMapper::RUDDER]   =  _("Rudder");
    lAxisStringsGUI[T_AxisMapper::THROTTLE] =  _("Throttle");
    lAxisStringsGUI[T_AxisMapper::FLAP]     =  _("Flap");
    lAxisStringsGUI[T_AxisMapper::SPOILER]  =  _("Spoiler");
    lAxisStringsGUI[T_AxisMapper::RETRACT]  =  _("Retract");
    lAxisStringsGUI[T_AxisMapper::PITCH]    =  _("Pitch");
    int size;
    AxisStringsGUI = T_GUI_Util::loadnames(lAxisStringsGUI, size);
  } 
 

  {   //for XML
    std::vector<std::string> lInputMethodStrings(T_TX_Interface::NUM_INPUTMETHODS);
    lInputMethodStrings[T_TX_Interface::eIM_keyboard] = "Keyboard";
    lInputMethodStrings[T_TX_Interface::eIM_mouse]    = "Mouse";
    lInputMethodStrings[T_TX_Interface::eIM_joystick] = "Joystick";
    lInputMethodStrings[T_TX_Interface::eIM_rctran]   = "RCTran";
    lInputMethodStrings[T_TX_Interface::eIM_audio]    = "Audio";
    lInputMethodStrings[T_TX_Interface::eIM_parallel] = "Parallel";
    lInputMethodStrings[T_TX_Interface::eIM_serial2]  = "Serial2";
    lInputMethodStrings[T_TX_Interface::eIM_rctran2]  = "RCTran2";
    lInputMethodStrings[T_TX_Interface::eIM_serpic]   = "FMSPIC";
    lInputMethodStrings[T_TX_Interface::eIM_mnav]     = "MNAV";
    lInputMethodStrings[T_TX_Interface::eIM_zhenhua]  = "ZhenHua";
    lInputMethodStrings[T_TX_Interface::eIM_CT6A]     = "CT6A";
    int size;
    InputMethodStrings = T_GUI_Util::loadnames(lInputMethodStrings, size);
  }
  {   // The same but possibly translated
    std::vector<std::string> lInputMethodStrings(T_TX_Interface::NUM_INPUTMETHODS);
    lInputMethodStrings[T_TX_Interface::eIM_keyboard] = _("Keyboard");
    lInputMethodStrings[T_TX_Interface::eIM_mouse]    = _("Mouse");
    lInputMethodStrings[T_TX_Interface::eIM_joystick] = _("Joystick");
    lInputMethodStrings[T_TX_Interface::eIM_rctran]   = _("RCTran");
    lInputMethodStrings[T_TX_Interface::eIM_audio]    = _("Audio");
    lInputMethodStrings[T_TX_Interface::eIM_parallel] = _("Parallel");
    lInputMethodStrings[T_TX_Interface::eIM_serial2]  = "Serial2";
    lInputMethodStrings[T_TX_Interface::eIM_rctran2]  = "RCTran2";
    lInputMethodStrings[T_TX_Interface::eIM_serpic]   = "FMSPIC";
    lInputMethodStrings[T_TX_Interface::eIM_mnav]     = "MNAV";
    lInputMethodStrings[T_TX_Interface::eIM_zhenhua]  = "ZhenHua";
    lInputMethodStrings[T_TX_Interface::eIM_CT6A]     = "CT6A";
    int size;     
    InputMethodStringsGUI = T_GUI_Util::loadnames(lInputMethodStrings, size);
  }
  {   // for XML
    std::vector<std::string> lActionButtonStrings(NUM_BUTTONACTION);
    lActionButtonStrings[NOTHING]      = "NOTHING";
    lActionButtonStrings[RESUME]       = "RESUME";
    lActionButtonStrings[PAUSE]        = "PAUSE";
    lActionButtonStrings[RESET]        = "RESET";
    lActionButtonStrings[INCTHROTTLE]  = "INCTHROTTLE";
    lActionButtonStrings[DECTHROTTLE]  = "DECTHROTTLE";
    lActionButtonStrings[ZOOMIN]       = "ZOOMIN";
    lActionButtonStrings[ZOOMIN]       = "ZOOMIN";
    int size;     
    ActionButtonStrings = T_GUI_Util::loadnames(lActionButtonStrings, size);
  }
  {   //for GUI, possibly translated
    std::vector<std::string> lActionButtonStringsGUI(NUM_BUTTONACTION);
    lActionButtonStringsGUI[NOTHING]      = _("Nothing");
    lActionButtonStringsGUI[RESUME]       = _("Resume");
    lActionButtonStringsGUI[PAUSE]        = _("Pause");
    lActionButtonStringsGUI[RESET]        = _("Reset");
    lActionButtonStringsGUI[INCTHROTTLE]  = _("Inc Throttle");
    lActionButtonStringsGUI[DECTHROTTLE]  = _("Dec Throttle");
    lActionButtonStringsGUI[ZOOMIN]       = _("Zoom In");
    lActionButtonStringsGUI[ZOOMOUT]      = _("Zoom Out");
    int size;     
    ActionButtonStringsGUI = T_GUI_Util::loadnames(lActionButtonStringsGUI, size);
  } 
  {   // for XML
    std::vector<std::string> lZoomControlStrings(NUM_ZOMCONTROL);
    lZoomControlStrings[KEYBOARD]    = "KEYBOARD";
    lZoomControlStrings[MOUSE]       = "MOUSE";
    int size;     
    ZoomControlStrings = T_GUI_Util::loadnames(lZoomControlStrings, size);
  }
  {   //for GUI, possibly translated
    std::vector<std::string> lZoomControlStrings(NUM_ZOMCONTROL);
    lZoomControlStrings[KEYBOARD]    = _("Keyboard");
    lZoomControlStrings[MOUSE]       = _("Mouse");
    int size;     
    ZoomControlStringsGUI = T_GUI_Util::loadnames(lZoomControlStrings, size);
  }
  { 
    std::vector<std::string> lMixerStringsXML(T_TX_Mixer::NUM_MIXERS);
    std::vector<std::string> lMixerStringsGUI(T_TX_Mixer::NUM_MIXERS);
    for (int i = 0; i < T_TX_Mixer::NUM_MIXERS; i++)
    {
      std::string s;
      std::stringstream out;
      out << (i+1);
      s = out.str();
      lMixerStringsXML[i] = "mixer" + s; 
      lMixerStringsGUI[i] = _("Mixer") + s; 
    }
    int size;     
    MixerStringsXML = T_GUI_Util::loadnames(lMixerStringsXML, size);
    MixerStringsGUI = T_GUI_Util::loadnames(lMixerStringsGUI, size);
  }
}

void TInputDev::closeJoystick()
{
  if (joy != (SDL_Joystick*)0)
  {
    SDL_JoystickClose(joy);
    joy = (SDL_Joystick*)0;
  }
}

int TInputDev::getJoystickNumAxes()
{
  #if TEST_WITHOUT_JOYSTICK == 0
  if (joy != NULL)
  {
    return SDL_JoystickNumAxes(joy);
  }
  else
  {
    return 0;
  }
  #else
  return SIMULATED_JOYSTICK_AXES;
  #endif
}

int TInputDev::getJoystickNumButtons()
{
  #if TEST_WITHOUT_JOYSTICK == 0
  if (joy != NULL)
  {
    return SDL_JoystickNumButtons(joy);
  }
  else
  {
    return 0;
  }
  #else
  return SIMULATED_JOYSTICK_BUTTONS;
  #endif
}
