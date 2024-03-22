/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2008 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2005, 2008, 2009, 2010 - Jan Reucker
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
#include "inputdev_software.h"
#include <SDL.h>
#include "../../defines.h"

#include "../../mod_misc/lib_conversions.h"

T_TX_InterfaceSoftware::T_TX_InterfaceSoftware(int method, int axesnum)
  :     input_method(method),
    numberOfAxes(axesnum)
{
#if DEBUG_TX_INTERFACE > 0
  printf("T_TX_InterfaceSoftware::T_TX_InterfaceSoftware(int method)\n");
#endif

  mixer = new T_TX_Mixer(this);

  switch (input_method)
  {
    case eIM_joystick:
      map = new T_AxisMapper(this);
      calib = new T_Calibration(this);
      break;
    
    case eIM_mouse:
      map = new T_AxisMapper(this);
      break;
    
    default:
      break;
  }
  
  for (int i = 0; i < TX_MAXAXIS; i++)
  {
    input_raw_data[i] = 0.0;
  }
}

T_TX_InterfaceSoftware::~T_TX_InterfaceSoftware()
{
#if DEBUG_TX_INTERFACE > 0
  printf("T_TX_InterfaceSoftware::~T_TX_InterfaceSoftware()\n");
#endif
  delete map;
  delete calib;
  delete mixer;
}

int T_TX_InterfaceSoftware::init(SimpleXMLTransfer* config)
{
#if DEBUG_TX_INTERFACE > 0
  printf("int T_TX_InterfaceSoftware::init(SimpleXMLTransfer* config)\n");
#endif

  int ret = 0;

  // initialize generic stuff
  T_TX_Interface::init(config);

  // initialize mixer
  switch (input_method)
  {
    case eIM_mouse:
      ret = mixer->init(config, "inputMethod.mouse");
      map->init(config, "inputMethod.mouse");
      break;

    case eIM_joystick:
      ret = mixer->init(config, "inputMethod.joystick");
      map->init(config, "inputMethod.joystick");
      calib->init(config, "inputMethod.joystick");
      break;

    case eIM_keyboard:
    default:
      ret = mixer->init(config, "inputMethod.keyboard");
      break;
  }

  return ret;
}

void T_TX_InterfaceSoftware::putBackIntoCfg(SimpleXMLTransfer* config)
{
#if DEBUG_TX_INTERFACE > 0
  printf("int T_TX_InterfaceSoftware::putBackIntoCfg(SimpleXMLTransfer* config)\n");
#endif
  T_TX_Interface::putBackIntoCfg(config);

  if (usesMixer())
  {
    mixer->putBackIntoCfg(config);
  }
  if (usesMapper())
  {
    map->putBackIntoCfg(config);
  }
  if (usesCalibration())
  {
    calib->putBackIntoCfg(config);
  }
}

void T_TX_InterfaceSoftware::getInputData(TSimInputs* inputs)
{
#if DEBUG_TX_INTERFACE > 1
  printf("void T_TX_InterfaceSoftware::getInputData(TSimInputs* inputs)\n");
#endif

  float calibrated[T_AxisMapper::NUM_AXISFUNCS];
  int   axisnum;
  
  // pre-initialize all axes with the values from the keyboard interface
  preInitFromKeyboard(inputs);

  calibrated[T_AxisMapper::NOTHING] = 0.0;
  calibrated[T_AxisMapper::ELEVATOR] = inputs->elevator;
  calibrated[T_AxisMapper::AILERON] = inputs->aileron;
  calibrated[T_AxisMapper::RUDDER] = inputs->rudder;
  calibrated[T_AxisMapper::THROTTLE] = inputs->throttle;
  calibrated[T_AxisMapper::FLAP] = inputs->flap;
  calibrated[T_AxisMapper::SPOILER] = inputs->spoiler;
  calibrated[T_AxisMapper::RETRACT] = inputs->retract;
  calibrated[T_AxisMapper::PITCH] = inputs->pitch;

  // now override all axes that are mapped to a "real" controller
    
  // evaluate calibrated input for all axis  

  if (input_method != eIM_keyboard)
  {
    if (usesCalibration())
    {
      axisnum    = map->func[T_AxisMapper::ELEVATOR];
      if (axisnum >= 0)
        calibrated[T_AxisMapper::ELEVATOR] = 
          calib->calibrate(axisnum, input_raw_data[axisnum]) * map->inv[T_AxisMapper::ELEVATOR];
      
      axisnum    = map->func[T_AxisMapper::AILERON];
      if (axisnum >= 0)
        calibrated[T_AxisMapper::AILERON] = 
          calib->calibrate(axisnum, input_raw_data[axisnum]) * map->inv[T_AxisMapper::AILERON];
      
      axisnum    = map->func[T_AxisMapper::RUDDER];
      if (axisnum >= 0)
        calibrated[T_AxisMapper::RUDDER] = 
          calib->calibrate(axisnum, input_raw_data[axisnum]) * map->inv[T_AxisMapper::RUDDER];
    
      axisnum    = map->func[T_AxisMapper::THROTTLE];
      if (axisnum >= 0)
        calibrated[T_AxisMapper::THROTTLE] = 
          calib->calibrate(axisnum, input_raw_data[axisnum]) * map->inv[T_AxisMapper::THROTTLE];

      axisnum    = map->func[T_AxisMapper::FLAP];
      if (axisnum >= 0)
        calibrated[T_AxisMapper::FLAP] = 
          calib->calibrate(axisnum, input_raw_data[axisnum]) * map->inv[T_AxisMapper::FLAP];
    
      axisnum    = map->func[T_AxisMapper::SPOILER];
      if (axisnum >= 0)
        calibrated[T_AxisMapper::SPOILER] = 
          calib->calibrate(axisnum, input_raw_data[axisnum]) * map->inv[T_AxisMapper::SPOILER];
    
      axisnum    = map->func[T_AxisMapper::RETRACT];
      if (axisnum >= 0)
        calibrated[T_AxisMapper::RETRACT] = 
          calib->calibrate(axisnum, input_raw_data[axisnum]) * map->inv[T_AxisMapper::RETRACT];
    
      axisnum    = map->func[T_AxisMapper::PITCH];
      if (axisnum >= 0)
        calibrated[T_AxisMapper::PITCH] = 
          calib->calibrate(axisnum, input_raw_data[axisnum]) * map->inv[T_AxisMapper::PITCH];
    }
    else  // doesn't use calibration
    {
      axisnum    = map->func[T_AxisMapper::ELEVATOR];
      if (axisnum >= 0)
        calibrated[T_AxisMapper::ELEVATOR] = 
          input_raw_data[axisnum] * map->inv[T_AxisMapper::ELEVATOR];
      
      axisnum    = map->func[T_AxisMapper::AILERON];
      if (axisnum >= 0)
        calibrated[T_AxisMapper::AILERON] = 
          input_raw_data[axisnum] * map->inv[T_AxisMapper::AILERON];
      
      axisnum    = map->func[T_AxisMapper::RUDDER];
      if (axisnum >= 0)
        calibrated[T_AxisMapper::RUDDER] = 
          input_raw_data[axisnum] * map->inv[T_AxisMapper::RUDDER];
    
      axisnum    = map->func[T_AxisMapper::THROTTLE];
      if (axisnum >= 0)
        calibrated[T_AxisMapper::THROTTLE] = 
          input_raw_data[axisnum] * map->inv[T_AxisMapper::THROTTLE];

      axisnum    = map->func[T_AxisMapper::FLAP];
      if (axisnum >= 0)
        calibrated[T_AxisMapper::FLAP] = 
          input_raw_data[axisnum] * map->inv[T_AxisMapper::FLAP];
    
      axisnum    = map->func[T_AxisMapper::SPOILER];
      if (axisnum >= 0)
        calibrated[T_AxisMapper::SPOILER] = 
          input_raw_data[axisnum] * map->inv[T_AxisMapper::SPOILER];
    
      axisnum    = map->func[T_AxisMapper::RETRACT];
      if (axisnum >= 0)
        calibrated[T_AxisMapper::RETRACT] = 
          input_raw_data[axisnum] * map->inv[T_AxisMapper::RETRACT];
    
      axisnum    = map->func[T_AxisMapper::PITCH];
      if (axisnum >= 0)
        calibrated[T_AxisMapper::PITCH] = 
          input_raw_data[axisnum] * map->inv[T_AxisMapper::PITCH];
    }
  }
  
  // apply expo, scaling and trim  
  calibrated[T_AxisMapper::ELEVATOR] = limit(mixer->mix_signed(
    calibrated[T_AxisMapper::ELEVATOR], T_AxisMapper::ELEVATOR));
  calibrated[T_AxisMapper::AILERON]  = limit(mixer->mix_signed(
    calibrated[T_AxisMapper::AILERON],  T_AxisMapper::AILERON));
  calibrated[T_AxisMapper::RUDDER]   = limit(mixer->mix_signed(
    calibrated[T_AxisMapper::RUDDER],   T_AxisMapper::RUDDER));
  calibrated[T_AxisMapper::THROTTLE] = limit_unsigned(mixer->mix_unsigned(
    calibrated[T_AxisMapper::THROTTLE], T_AxisMapper::THROTTLE));
  calibrated[T_AxisMapper::FLAP]     = limit(mixer->mix_signed(
    calibrated[T_AxisMapper::FLAP],     T_AxisMapper::FLAP));
  calibrated[T_AxisMapper::SPOILER]  = limit_unsigned(mixer->mix_unsigned(
    calibrated[T_AxisMapper::SPOILER],  T_AxisMapper::SPOILER));
  calibrated[T_AxisMapper::RETRACT]  = limit_unsigned(mixer->mix_unsigned(
    calibrated[T_AxisMapper::RETRACT],  T_AxisMapper::RETRACT));
  calibrated[T_AxisMapper::PITCH]    = limit(mixer->mix_signed(
    calibrated[T_AxisMapper::PITCH],    T_AxisMapper::PITCH));

  // further apply mixers  
  inputs->elevator = limit(mixer->mix_mixer(calibrated, T_AxisMapper::ELEVATOR));
  inputs->aileron  = limit(mixer->mix_mixer(calibrated, T_AxisMapper::AILERON));
  inputs->rudder   = limit(mixer->mix_mixer(calibrated, T_AxisMapper::RUDDER));
  inputs->throttle = limit_unsigned(mixer->mix_mixer(calibrated, T_AxisMapper::THROTTLE));
  inputs->flap     = limit(mixer->mix_mixer(calibrated, T_AxisMapper::FLAP));
  inputs->spoiler  = limit_unsigned(mixer->mix_mixer(calibrated, T_AxisMapper::SPOILER));
  inputs->retract  = limit_unsigned(mixer->mix_mixer(calibrated, T_AxisMapper::RETRACT));
  inputs->pitch    = limit(mixer->mix_mixer(calibrated, T_AxisMapper::PITCH));
}


void T_TX_InterfaceSoftware::getRawData(float *dest)
{
  int axes = getNumAxes();
  
  if (axes > TX_MAXAXIS)
  {
    axes = TX_MAXAXIS;
  }
  for (int i = 0; i < axes; i++)
  {
    *(dest + i) = input_raw_data[i];
  }
}

void T_TX_InterfaceSoftware::setAxis(int axis, const float x)
{
  input_raw_data[axis] = limit(x);
}


int T_TX_InterfaceSoftware::getDeviceList(std::vector<std::string>& Devices)
{
  Devices.erase(Devices.begin(), Devices.end());

  #if TEST_WITHOUT_JOYSTICK > 0
  for (int i = 0; i < TEST_WITHOUT_JOYSTICK; i++)
  {
    char number[7];
    std::string name;
    
    snprintf(number, 6, "%2d - ", i);
    name = number;
    name += "fake joystick";
    Devices.push_back(name);
  }
  #else
  for (int i = 0; i < SDL_NumJoysticks(); i++)
  {
    std::string name;
    
    if (SDL_JoystickName(i) != NULL)
    {
      name = SDL_JoystickName(i);
    }
    else
    {
      name = "not available (removed?)";
      std::cerr << "CGUICtrlGeneralDialog::rebuildJoystickComboBox(): error:" << std::endl;
      std::cerr << "  Joystick " << i << " is no longer available (removed after starting CRRCsim?)." << std::endl;
    }
    Devices.push_back(name);
  }
  #endif
  
  return Devices.size();
}

