/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2008, 2010 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2005, 2008, 2010 - Jan Reucker
 *   Copyright (C) 2007 - Martin Herrmann
 *   Copyright (C) 2006 - Todd Templeton
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
#include "../global.h"
#include "inputdev.h"
#include "../GUI/util.h"
#include "../mod_misc/lib_conversions.h"
#include <cstdlib>
#include <string.h>
#include <math.h>

// 1/exp(exp(1)), needed for mix_exp
#ifndef M_1_E_E
# define M_1_E_E 0.065988
#endif

// exp(1), needed for mix_exp
#ifndef M_E
# define M_E 2.7182818284590452354
#endif

/// Don't change the order of the following strings unless you know exactly what
/// you're doing!

/**
 * Strings ordered like radio type enums in T_TX_Interface
 */
const char* RadioTypeStrings[] = {"Futaba", "Airtronics", "Hitec", "JR", "Cockpit", "Walkera", "Custom"};


int T_TX_Interface::init(SimpleXMLTransfer* config)
{
#if DEBUG_TX_INTERFACE > 0
  printf("int T_TX_Interface::init(SimpleXMLTransfer* config)\n");
#endif

  return(0);
}

std::string T_TX_Interface::getErrMsg() 
{
  return(errMsg);
};


void T_TX_Interface::getInputData(TSimInputs* inputs)
{
#if DEBUG_TX_INTERFACE > 1
  printf("void T_TX_Interface::getInputData(TSimInputs* inputs)\n");
#endif
}

T_TX_Interface::T_TX_Interface()
  : mixer(NULL), calib(NULL), map(NULL), errMsg(""),
    keyb_retract_limited(-0.5, 1)
{
#if DEBUG_TX_INTERFACE > 0
  printf("T_TX_Interface::T_TX_Interface\n");
#endif

  reset();
}

T_TX_Interface::~T_TX_Interface()
{
#if DEBUG_TX_INTERFACE > 0
  printf("T_TX_Interface::~T_TX_Interface\n");
#endif
}

void T_TX_Interface::putBackIntoCfg(SimpleXMLTransfer* config)
{
}

float T_TX_Interface::limit(float in)
{
  if (in < -0.5)
  {
    return -0.5;
  }
  else if (in > 0.5)
  {
    return 0.5;
  }
  else
  {
    return in;
  }
}

/** Limit unsigned axis values
 *
 *  Limits unsigned axis values and suppresses jitter just above zero.
 */
float T_TX_Interface::limit_unsigned(float in)
{
  float out;
  
  if (in > 1.0)
  {
    out = 1.0;
  }
  else if (in < 0.05)
  {
    out = 0.0;
  } 
  else
  {
    out = in;
  }

  return out;
}

void T_TX_Interface::CalibMixMapValues(TSimInputs* inputs, float* myArrayOfValues)
{
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
  
  // evaluate calibrated, scaled and trimmed input for all axis    

  axisnum    = map->func[T_AxisMapper::ELEVATOR];
  if (axisnum >= 0)
    calibrated[T_AxisMapper::ELEVATOR] =
      calib->calibrate(axisnum, myArrayOfValues[axisnum]) * map->inv[T_AxisMapper::ELEVATOR];
  
  axisnum    = map->func[T_AxisMapper::AILERON];
  if (axisnum >= 0)
    calibrated[T_AxisMapper::AILERON] =
      calib->calibrate(axisnum, myArrayOfValues[axisnum]) * map->inv[T_AxisMapper::AILERON];

  axisnum    = map->func[T_AxisMapper::RUDDER];
  if (axisnum >= 0)
    calibrated[T_AxisMapper::RUDDER] =
      calib->calibrate(axisnum, myArrayOfValues[axisnum]) * map->inv[T_AxisMapper::RUDDER];

  axisnum    = map->func[T_AxisMapper::THROTTLE];
  if (axisnum >= 0)
    calibrated[T_AxisMapper::THROTTLE] = 
      calib->calibrate(axisnum, myArrayOfValues[axisnum]) * map->inv[T_AxisMapper::THROTTLE];

  axisnum    = map->func[T_AxisMapper::FLAP];
  if (axisnum >= 0)
    calibrated[T_AxisMapper::FLAP] = 
      calib->calibrate(axisnum, myArrayOfValues[axisnum]) * map->inv[T_AxisMapper::FLAP];

  axisnum    = map->func[T_AxisMapper::SPOILER];
  if (axisnum >= 0)
    calibrated[T_AxisMapper::SPOILER] = 
      calib->calibrate(axisnum, myArrayOfValues[axisnum]) * map->inv[T_AxisMapper::SPOILER];
 
  axisnum    = map->func[T_AxisMapper::RETRACT];
  if (axisnum >= 0)
    calibrated[T_AxisMapper::RETRACT] = 
      calib->calibrate(axisnum, myArrayOfValues[axisnum]) * map->inv[T_AxisMapper::RETRACT];

  axisnum    = map->func[T_AxisMapper::PITCH];
  if (axisnum >= 0)
    calibrated[T_AxisMapper::PITCH] = 
      calib->calibrate(axisnum, myArrayOfValues[axisnum]) * map->inv[T_AxisMapper::PITCH];

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


void T_TX_Interface::toggleRetract()
{
  if (keyb_retract_input > 0.0)
  {
    keyb_retract_input = -0.5f;
  }
  else
  {
    keyb_retract_input = 0.5f;
  }
}

void T_TX_Interface::toggleSpoiler()
{
  if (keyb_spoiler_input > 0.0)
  {
    keyb_spoiler_input = -0.5f;
  }
  else
  {
    keyb_spoiler_input = 0.5f;
  }
}

void T_TX_Interface::reset()
{
  centerControls();
  keyb_throttle_input = -0.5f;
  keyb_flap_input = 0.0f;
  keyb_spoiler_input = -0.5f;
  keyb_spoiler_limited.init(-0.5f, 1.0f);
  keyb_retract_input = -0.5f;
  keyb_retract_limited.init(-0.5f, 1.0f);
  keyb_pitch_input = 0.0f;
}

void T_TX_Interface::increase_throttle()
{
  float tmp = keyb_throttle_input + 0.1;
  keyb_throttle_input = limit(tmp);
}

void T_TX_Interface::decrease_throttle()
{
  float tmp = keyb_throttle_input - 0.1;
  keyb_throttle_input = limit(tmp);
}

void T_TX_Interface::move_aileron(const float x)
{
  float tmp = keyb_aileron_input + x;
  keyb_aileron_input = limit(tmp);
}

void T_TX_Interface::move_rudder(const float x)
{
  float tmp = keyb_rudder_input + x;
  keyb_rudder_input = limit(tmp);
}

void T_TX_Interface::move_elevator(const float x)
{
  float tmp = keyb_elevator_input + x;
  keyb_elevator_input = limit(tmp);
}

void T_TX_Interface::move_flap(const float x)
{
  float tmp = keyb_flap_input + x;
  keyb_flap_input = limit(tmp);
}

void T_TX_Interface::move_spoiler(const float x)
{
  float tmp = keyb_spoiler_input + x;
  keyb_spoiler_input = limit(tmp);
}

void T_TX_Interface::move_retract(const float x)
{
  float tmp = keyb_retract_input + x;
  keyb_retract_input = limit(tmp);
}

void T_TX_Interface::move_pitch(const float x)
{
  float tmp = keyb_pitch_input + x;
  keyb_pitch_input = limit(tmp);
}

void T_TX_Interface::centerControls()
{
  keyb_elevator_input = 0.0;
  keyb_aileron_input = 0.0;
  keyb_rudder_input = 0.0;
}

void T_TX_Interface::update(double dt)
{
  keyb_retract_limited.step(dt, keyb_retract_input);
  keyb_spoiler_limited.step(dt, keyb_spoiler_input);
}



// --- Implementation of class T_TX_Mixer -------------------------------------

/** \brief Create a mixer object
 *
 *  The mixer will be initialized with default values: scaling of all
 *  axes is set to 1.0, offsets of throttle, spoiler, retract are set to 0.5.
 *  Mixers are enabled but not effective.
 */
T_TX_Mixer::T_TX_Mixer(T_TX_Interface *parent) 
  : child_in_cfg(""), iface(parent)
{
  baseInit();
#if DEBUG_TX_INTERFACE > 0
  printf("T_TX_Mixer::T_TX_Mixer()\n");
#endif
}


/** \brief Create and initialize a mixer object
 *
 *  The mixer will be initialized from the given child
 *  in the SimpleXMLTransfer. This does the same as if you
 *  would call <code>T_TX_Mixer::init(cfg, child)</code>
 *  after calling the empty constructor.
 *
 *  \param cfg Pointer to an XML tree containing the config info.
 *  \param child The name of the child which holds the mixer info.
 */
T_TX_Mixer::T_TX_Mixer(T_TX_Interface *parent, 
                       SimpleXMLTransfer* cfg, std::string child)
  : child_in_cfg(""), iface(parent)
{
  baseInit();
#if DEBUG_TX_INTERFACE > 0
  printf("T_TX_Mixer::T_TX_Mixer(cfg, child)\n");
#endif
  init(cfg, child);
}


/** \brief Destroy the mixer.
 *
 *  Right now the dtor is only provided for debugging purposes.
 */
T_TX_Mixer::~T_TX_Mixer()
{
#if DEBUG_TX_INTERFACE > 0
  printf("T_TX_Mixer::~T_TX_Mixer()\n");
#endif
}


/** \brief Initialize all variables
 */
void T_TX_Mixer::baseInit()
{
  enabled = 1;
  dr_enabled = 0; // dual-rate off

  for (int i = 0; i < T_AxisMapper::NUM_AXISFUNCS; i++)
  {
    trim_val[i]  = 0.0;
    nrate_val[i] = 1.0;
    srate_val[i] = 1.0;
    exp_val[i]   = 0.0;
 
    mtravel_val[i] = -0.5;
    ptravel_val[i] = 0.5;
  }

  for (int i = 0; i < T_TX_Mixer::NUM_MIXERS; i++)
  {
    mixer_enabled[i] = 0;
    mixer_src[i] = mixer_dst[i] = T_AxisMapper::NOTHING;
    mixer_val[i] = 0.0;
  }
}


/** \brief Initialize the mixer from a config file.
 *
 *  The mixer object will be initialized from the given config file.
 *  This file may contain more than one branch with interface settings,
 *  so the name of the child has to be specified.
 */
int T_TX_Mixer::init(SimpleXMLTransfer* cfg, std::string childname)
{
#if DEBUG_TX_INTERFACE > 0
  printf("T_TX_Mixer::init(cfg, child)\n");
  printf(" <-- %s\n", childname.c_str());
#endif

  // store the child's name for writing back the settings
  child_in_cfg = childname;

  return reinit(cfg, false);
}


/** \brief Re-initialize the mixer from a config file.
 *
 *  The mixer object will be re-initialized from the given config file.
 *  If fIsChild = true the file only contains the mixer child.
 *  The interface, hence the name of the interface child, are unchanged.
 */
int T_TX_Mixer::reinit(SimpleXMLTransfer* cfg, bool fIsChild)
{
  SimpleXMLTransfer* mixer;

  if (fIsChild)
  {
#if DEBUG_TX_INTERFACE > 0
    printf("T_TX_Mixer::reinit(cfg, fIsChild)\n");
#endif
    mixer = cfg;
  }
  else
  {
    SimpleXMLTransfer* inter = cfg->getChild(child_in_cfg, true);
    mixer = inter->getChild("mixer", true);
  }

  printf("Loading mixer settings from %s:\n", child_in_cfg.c_str());

  try
  {
    enabled = mixer->attributeAsInt("enabled", 1);
    dr_enabled = mixer->attributeAsInt("dr_enabled", 0);
 
    for (int i = T_AxisMapper::AILERON; i <= T_AxisMapper::PITCH; i++)
    { 
      SimpleXMLTransfer* item = mixer->getChild(Global::inputDev->AxisStringsXML[i], true);

      trim_val[i]   = item->getDouble("trim",   0.0);
      nrate_val[i]  = item->getDouble("nrate",  1.0);
      srate_val[i]  = item->getDouble("srate",  1.0);
      exp_val[i]    = item->getDouble("exp",    0.0);

      mtravel_val[i] = item->getDouble("mtravel", -0.5);
      ptravel_val[i] = item->getDouble("ptravel", 0.5);
    }
      
    for (int i = 0; i < T_TX_Mixer::NUM_MIXERS; i++)
    { 
      SimpleXMLTransfer* item = mixer->getChild(Global::inputDev->MixerStringsXML[i], true);
      
      mixer_enabled[i] = item->attributeAsInt("enabled", 0);
      mixer_src[i]     = item->getDouble("src", T_AxisMapper::NOTHING);
      mixer_dst[i]     = item->getDouble("dst", T_AxisMapper::NOTHING);
      mixer_val[i]     = item->getDouble("val", 0.0);
    }
  }
  catch (XMLException e)
  {
    errMsg = e.what();
    return 1;
  }

  return 0;
}


/** \brief Transfers all settings back to the config file.
 *
 *  The mixer settings will be stored in the same branch of the config
 *  file that they were read from on initialization.
 *
 *  \param config Pointer to the config file's SimpleXMLTransfer.
 */
void T_TX_Mixer::putBackIntoCfg(SimpleXMLTransfer* config)
{
#if DEBUG_TX_INTERFACE > 0
  printf("T_TX_Mixer::putBackIntoCfg(SimpleXMLTransfer* config)\n");
  printf(" --> %s\n", child_in_cfg.c_str());
#endif

  SimpleXMLTransfer* inter = config->getChild(child_in_cfg);
  SimpleXMLTransfer* mixer = inter->getChild("mixer");
  SimpleXMLTransfer* item;

  mixer->setAttributeOverwrite("enabled", enabled);
  mixer->setAttributeOverwrite("dr_enabled", dr_enabled);

  item = mixer->getChild(Global::inputDev->AxisStringsXML[T_AxisMapper::AILERON], true);
  item->setAttributeOverwrite("trim",  doubleToString(trim_val[T_AxisMapper::AILERON]));
  item->setAttributeOverwrite("nrate", doubleToString(nrate_val[T_AxisMapper::AILERON]));
  item->setAttributeOverwrite("srate", doubleToString(srate_val[T_AxisMapper::AILERON]));
  item->setAttributeOverwrite("exp",   doubleToString(exp_val[T_AxisMapper::AILERON]));

  item = mixer->getChild(Global::inputDev->AxisStringsXML[T_AxisMapper::ELEVATOR], true);
  item->setAttributeOverwrite("trim",  doubleToString(trim_val[T_AxisMapper::ELEVATOR]));
  item->setAttributeOverwrite("nrate", doubleToString(nrate_val[T_AxisMapper::ELEVATOR]));
  item->setAttributeOverwrite("srate", doubleToString(srate_val[T_AxisMapper::ELEVATOR]));
  item->setAttributeOverwrite("exp",   doubleToString(exp_val[T_AxisMapper::ELEVATOR]));

  item = mixer->getChild(Global::inputDev->AxisStringsXML[T_AxisMapper::RUDDER], true);
  item->setAttributeOverwrite("trim",  doubleToString(trim_val[T_AxisMapper::RUDDER]));
  item->setAttributeOverwrite("nrate", doubleToString(nrate_val[T_AxisMapper::RUDDER]));
  item->setAttributeOverwrite("srate", doubleToString(srate_val[T_AxisMapper::RUDDER]));
  item->setAttributeOverwrite("exp",   doubleToString(exp_val[T_AxisMapper::RUDDER]));

  item = mixer->getChild(Global::inputDev->AxisStringsXML[T_AxisMapper::FLAP], true);
  item->setAttributeOverwrite("trim",    doubleToString(trim_val[T_AxisMapper::FLAP]));
  item->setAttributeOverwrite("mtravel", doubleToString(mtravel_val[T_AxisMapper::FLAP]));
  item->setAttributeOverwrite("ptravel", doubleToString(ptravel_val[T_AxisMapper::FLAP]));
  
  for (int i = 0; i < T_TX_Mixer::NUM_MIXERS; i++)
  {
    item = mixer->getChild(Global::inputDev->MixerStringsXML[i], true);
    item->setAttributeOverwrite("enabled", mixer_enabled[i]);
    item->setAttributeOverwrite("src", mixer_src[i]);
    item->setAttributeOverwrite("dst", mixer_dst[i]);
    item->setAttributeOverwrite("val", doubleToString(mixer_val[i]));
  }
}


/** \brief Apply exponential component to input.
 *
 *  \param x Linear input signal (-1.0 ... 1.0).
 *  \param p Percentage of exponential component to be applied to input (0.0 ... 1.0).
 */
float T_TX_Mixer::mix_exp(const float x, const float p) const
{
  return x * (1.0 + p * (exp(fabs(x) * M_E) * M_1_E_E - 1.0));
}


/** \brief Mix a centered axis
 *
 *  Mixes an axis which delivers positive and negative values
 * (e.g. aileron, elevator, ...)
 *
 * \param in        input value
 * \param function  One of the enum values defined in T_AxisMapper
 * \return          Mixed value
 */
float T_TX_Mixer::mix_signed(float in, int function) const
{
  if (enabled)
  {
    float tmp;
    float rate = dr_enabled ? srate_val[function] : nrate_val[function];
    float travel = in > 0.0 ? 
      ptravel_val[function] - trim_val[function] : trim_val[function] - mtravel_val[function];

    // "in" value is always in the range -0.5..+0.5
    // trim (offset) is added after applying exponential, travel & rate
    tmp = travel * rate * mix_exp(2.0*in, exp_val[function]) + trim_val[function];
    return tmp;
  }
  else
  {
    return in;
  }
}


/** \brief Mix a positive-only axis
 *
 *  Mixes an axis which delivers only positive values
 * (e.g. throttle, spoiler, retract)
 *
 * \param in        input value
 * \param function  One of the enum values defined in T_AxisMapper
 * \return          Mixed value
 */
float T_TX_Mixer::mix_unsigned(float in, int function) const
{
  if (enabled)
  {
    float tmp;
    float rate = nrate_val[function];
    float travel = in > 0.0 ? 
      ptravel_val[function] - trim_val[function] : trim_val[function] - mtravel_val[function];

    // "in" value is always in the range -0.5..+0.5
    // a default "-travel" offset is added to convert output to range 0..(ptravel-mtravel)
    // trim (offset) is added after applying exponential and rate
    tmp = travel * rate * mix_exp(2.0*in, exp_val[function]) + trim_val[function]
          - mtravel_val[function];
    return tmp;
  }
  else
  {
    return in + 0.5;
  }
}


/** \brief Mix-in mixers effect
 *
 *  Add mixers contribution to an axis
 * (e.g. aileron, elevator, ...)
 *
 * \param in        input value
 * \param function  One of the enum values defined in T_AxisMapper
 * \return          Mixed value
 */
float T_TX_Mixer::mix_mixer(float* in, int function) const
{
  if (enabled)
  {
    float tmp = in[function];

    for (int i = 0; i < T_TX_Mixer::NUM_MIXERS; i++)
      if (function == mixer_dst[i])
      { 
        tmp += mixer_enabled[i] * mixer_val[i] * in[mixer_src[i]];
      }
    return tmp;
  }
  else
  {
    return in[function];
  }
}


/** \brief Get current error message
 *
 *  Returns the current error message (if any).
 */
std::string T_TX_Mixer::getErrMsg() 
{
  return(errMsg);
};


// --- Implementation of class T_AxisMapper -------------------------

T_AxisMapper::T_AxisMapper(T_TX_Interface *parent)
  : child_in_cfg(""), radio_type(0), iface(parent)
{
  for (int i = 0; i < T_AxisMapper::NUM_AXISFUNCS; i++)
  {
    func[i]   = i;
    inv[i]    = 1;
    c_func[i] = i;
    c_inv[i]  = 1;
  }
}

T_AxisMapper::T_AxisMapper(T_TX_Interface *parent,
                           SimpleXMLTransfer* cfgfile,
                           std::string child)
  : child_in_cfg(""), radio_type(0), iface(parent)
{
  for (int i = 0; i < T_AxisMapper::NUM_AXISFUNCS; i++)
  {
    func[i]   = i;
    inv[i]    = 1;
    c_func[i] = i;
    c_inv[i]  = 1;
  }
  init(cfgfile, child);
}

void T_AxisMapper::init(SimpleXMLTransfer* cfgfile, std::string childname)
{
#if DEBUG_TX_INTERFACE > 0
  printf("T_AxisMapper::init(cfg, child)\n");
  printf(" <-- %s\n", childname.c_str());
#endif
  SimpleXMLTransfer* inter;
  SimpleXMLTransfer* bindings;
  SimpleXMLTransfer* group;
  SimpleXMLTransfer* item;
  
  child_in_cfg = childname;

  // try to load config
  try
  {
    inter = cfgfile->getChild(childname, true);
    bindings  = inter->getChild("bindings", true);
    group     = bindings->getChild("axes", true);
  
    for (int i = T_AxisMapper::AILERON; i <= T_AxisMapper::PITCH; i++)
    {
      int default_axis = -1;
      float default_polarity = 1.0;

      // special handling for some default values
      if (i == T_AxisMapper::AILERON)
      {
        default_axis = 0;
      }
      else if (i == T_AxisMapper::ELEVATOR)
      {
        default_axis = 1;
        if (iface->inputMethod() != T_TX_Interface::eIM_joystick)
        {
          default_polarity = -1.0;
        }
      }
      
      item = group->getChild(Global::inputDev->AxisStringsXML[i], true);
      c_func[i] = item->attributeAsInt("axis", default_axis);
      c_inv[i]  = item->attributeAsDouble("polarity", default_polarity);
    }

    std::string radio = 
      strU(bindings->attribute("radio_type", RadioTypeStrings[CUSTOM]));

    for (int n=0; n < NR_OF_RADIO_TYPES; n++)
    {
      if (radio.compare(strU(RadioTypeStrings[n])) == 0)
      {
        setRadioType(n);
      }
    }
  }
  catch (XMLException e)
  {
    fprintf(stderr, "*** T_AxisMapper: XMLException: %s\n", e.what());
  }
}


void T_AxisMapper::putBackIntoCfg(SimpleXMLTransfer* cfgfile)
{
#if DEBUG_TX_INTERFACE > 0
  printf("T_AxisMapper::putBackIntoCfg(SimpleXMLTransfer* config)\n");
  printf(" --> %s\n", child_in_cfg.c_str());
#endif
  SimpleXMLTransfer* item;
  SimpleXMLTransfer* group;
  SimpleXMLTransfer* item2;

  try
  {
    item = cfgfile->getChild(child_in_cfg);
    group = item->getChild("bindings.axes");

    for (int i = T_AxisMapper::AILERON; i <= T_AxisMapper::PITCH; i++)
    {
      item2 = group->getChild(Global::inputDev->AxisStringsXML[i], true);
      item2->setAttributeOverwrite("axis", c_func[i]);
      item2->setAttributeOverwrite("polarity", doubleToString(c_inv[i]));
    }
  
    item2 = item->getChild("bindings");
    item2->setAttributeOverwrite("radio_type", RadioTypeStrings[radio_type]);
  }
  catch (XMLException e)
  {
    fprintf(stderr, "*** T_AxisMapper: XMLException: %s\n", e.what());
  }
}


void T_AxisMapper::setRadioType(int rtype)
{
  printf ("mapper set to radio type %d\n", rtype);
  int   saved[T_AxisMapper::NUM_AXISFUNCS];
  float saved_i[T_AxisMapper::NUM_AXISFUNCS];
  
  for (int i = 0; i < T_AxisMapper::NUM_AXISFUNCS; i++)
  {
    saved[i] = func[i];
    saved_i[i] = inv[i];
    func[i] = i;
    inv[i] = 1.0;
  }

  switch (rtype)
  {
    case AIRTRONICS:
      func[T_AxisMapper::ELEVATOR]  = 0;
      func[T_AxisMapper::AILERON]   = 1;
      func[T_AxisMapper::THROTTLE]  = 2;
      func[T_AxisMapper::RUDDER]    = 3;
      radio_type = rtype;
      break;
    
    case JR:
      func[T_AxisMapper::ELEVATOR]  = 2;
      func[T_AxisMapper::AILERON]   = 1;
      func[T_AxisMapper::THROTTLE]  = 0;
      func[T_AxisMapper::RUDDER]    = 3;
      radio_type = rtype;
      break;
    
    case COCKPIT:
      func[T_AxisMapper::ELEVATOR]  = 1;
      func[T_AxisMapper::AILERON]   = 0;
      func[T_AxisMapper::THROTTLE]  = 3;
      func[T_AxisMapper::RUDDER]    = 2;
      radio_type = rtype;
      break;
    
    case FUTABA:
    case HITEC:
      func[T_AxisMapper::ELEVATOR]  = 1;
      func[T_AxisMapper::AILERON]   = 0;
      func[T_AxisMapper::THROTTLE]  = 2;
      func[T_AxisMapper::RUDDER]    = 3;
      radio_type = rtype;
      break;

    case WALKERA:
      func[T_AxisMapper::AILERON] =2; inv[T_AxisMapper::AILERON] =-1;
      func[T_AxisMapper::ELEVATOR]=1; inv[T_AxisMapper::ELEVATOR]=-1;
      func[T_AxisMapper::RUDDER]  =3; inv[T_AxisMapper::RUDDER  ]= 1;
      func[T_AxisMapper::THROTTLE]=0; inv[T_AxisMapper::THROTTLE]= 1;
      radio_type = rtype;
      break;
    
    case CUSTOM:
      for (int i = 0; i < T_AxisMapper::NUM_AXISFUNCS; i++)
      {
        func[i] = c_func[i];
        inv[i]  = c_inv[i];
      }
      radio_type = rtype;
      break;
    
    default:
      // don't change anything
      for (int i = 0; i < T_AxisMapper::NUM_AXISFUNCS; i++)
      {
        func[i] = saved[i];
        inv[i]  = saved_i[i];
      }
      break;
  }
}

void T_AxisMapper::saveToCustom()
{
#if DEBUG_TX_INTERFACE > 0
  printf("T_AxisMapper::saveToCustom()\n");
#endif
  int numAxis = iface->getNumAxes();
  for (int i = 0; i < T_AxisMapper::NUM_AXISFUNCS; i++)
  {
    // only store a value if it is in the valid range
    // for the current interface
    if (func[i] < numAxis)
    {
      c_func[i] = func[i];
      c_inv[i]  = inv[i];
    }
  }
}


// --- Implementation of class T_Calibration -------------------------
T_Calibration::T_Calibration(T_TX_Interface *parent)
  : child_in_cfg(""), iface(parent)
{
#if DEBUG_TX_INTERFACE > 0
  printf("T_Calibration::T_Calibration()\n");
#endif
  for (int i = 0; i < TX_MAXAXIS; i++)
  {
    val_min[i] = -1.0;
    val_mid[i] =  0.0;
    val_max[i] =  1.0;
  }
}

T_Calibration::T_Calibration(T_TX_Interface *parent,
                             SimpleXMLTransfer* cfg,
                             std::string child)
  : child_in_cfg(""), iface(parent)
{
#if DEBUG_TX_INTERFACE > 0
  printf("T_Calibration::T_Calibration(cfg, child)\n");
#endif
  for (int i = 0; i < TX_MAXAXIS; i++)
  {
    val_min[i] = -1.0;
    val_mid[i] =  0.0;
    val_max[i] =  1.0;
  }
  init(cfg, child);
}

void T_Calibration::init(SimpleXMLTransfer* cfgfile,
                         std::string childname)
{
#if DEBUG_TX_INTERFACE > 0
  printf("T_Calibration::init(cfg, child)\n");
  printf(" <-- %s\n", childname.c_str());
#endif
  int size;
  SimpleXMLTransfer* item;
  SimpleXMLTransfer* group;
  SimpleXMLTransfer* item2;
  
  child_in_cfg = childname;

  // try to load config
  printf("Loading calibration settings from %s:\n", childname.c_str());
  try
  {
    item = cfgfile->getChild(childname, true);
    group = item->getChild("calibration", true);
    int nVer = group->getInt("version", 1);
  
    size  = group->getChildCount();
    if (size > TX_MAXAXIS)
      size = TX_MAXAXIS;
    for (int n = 0; n < size; n++)
    {
      item2 = group->getChildAt(n);
      switch (nVer)
      {
        case 1:
          {
            float scale = item2->getDouble("scale", 1.0);
            float off = item2->getDouble("offset", 0.0);
            // old:   out = scale * in + offset
            val_min[n] = (-0.5 - off) / scale;
            val_max[n] = ( 0.5 - off) / scale;
            val_mid[n] = 0.5 * (val_min[n] + val_max[n]);
            printf("  (1)");
          }
          break;
          
        case 2:      
          val_min[n] = item2->getDouble("val_min", -1.0);
          val_mid[n] = item2->getDouble("val_mid",  0.0);
          val_max[n] = item2->getDouble("val_max",  1.0);
          printf("  (2)");
          break;
      }
      printf(" axis=%i val_min=%f val_mid=%f val_max=%f\n", n, val_min[n], val_mid[n], val_max[n]);
    }
  }
  catch (XMLException e)
  {
    fprintf(stderr, "*** T_Calibration::init(): %s\n", e.what());
  }
}


void T_Calibration::putBackIntoCfg(SimpleXMLTransfer* cfgfile)
{
#if DEBUG_TX_INTERFACE > 0
  printf("T_Calibration::putBackIntoCfg(cfg)\n");
  printf(" --> %s\n", child_in_cfg.c_str());
#endif
  int size;
  SimpleXMLTransfer* item;
  SimpleXMLTransfer* group;
  SimpleXMLTransfer* item2;

  item = cfgfile->getChild(child_in_cfg);
  group = item->getChild("calibration");
  group->setAttributeOverwrite("version", "2");

  // clean list
  size = group->getChildCount();
  for (int n = 0; n < size; n++)
  {
    item2 = group->getChildAt(0);
    group->removeChildAt(0);
    delete item2;
  }
  // create new list
  for (int n = 0; n < TX_MAXAXIS; n++)
  {
    item2 = new SimpleXMLTransfer();
    item2->setName("axis");
    item2->addAttribute("val_min", doubleToString(val_min[n]));
    item2->addAttribute("val_mid", doubleToString(val_mid[n]));
    item2->addAttribute("val_max", doubleToString(val_max[n]));
    group->addChild(item2);
  }
}


float T_Calibration::calibrate(int axis, float raw)
{
  float val = raw - val_mid[axis];
  if (val > 0)
    return(+0.5 * val / (val_max[axis] - val_mid[axis]));
  else
    return(-0.5 * val / (val_min[axis] - val_mid[axis]));
}

void T_Calibration::PrintSettings(int axis)
{
  printf("  axis=%i, min=%f, mid=%f, max=%f\n",
         axis, val_min[axis], val_mid[axis], val_max[axis]);
}
