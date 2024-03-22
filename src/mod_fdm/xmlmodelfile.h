/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2006, 2008 - Jens Wilhelm Wulf (original author)
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
#ifndef XMLMODELFILE_H
# define XMLMODELFILE_H

# include <string>
# include "../mod_misc/SimpleXMLTransfer.h"
# include "fdm_inputs.h"

/**
 * This reflects some parts of the specification of an xml file describing an airplane.
 * 
 * For example, an airplane may contain alternatives as far as 
 *   - graphical representation
 *   - mass (distribution), power system, sound
 * are concerned. This namespace contains methods to tell which grapics and which 
 * 'config' is to be used. This information (user preferences) should not be included 
 * in the file itself! It is written into the in-memory-copy by the application (which 
 * gets it from some user-defined configuration file or whereever).
 * 
 * @author Jens Wilhelm Wulf
 */
namespace XMLModelFile
{
  /**
   * Inserts a list of options to choose from.
   * Returns true if there is something to choose from.
   * 
   * This only copies data from somewhere to somewhere else to make some automatic
   * processing easier.
   *
   * @author Jens Wilhelm Wulf
   */
  bool ListOptions(SimpleXMLTransfer* xml);

  /**
   * Returns the graphics entry to use.
   *
   * @author Jens Wilhelm Wulf
   */
  SimpleXMLTransfer* getGraphics(SimpleXMLTransfer* xml);

  /**
   * Returns the config entry to use.
   *
   * @author Jens Wilhelm Wulf
   */
  SimpleXMLTransfer* getConfig(SimpleXMLTransfer* xml);
   
  /**
   * Returns a pointer to launch presets
   */
  SimpleXMLTransfer* getLaunchPresets(SimpleXMLTransfer* xml);
   
  /**
   * Returns a pointer to mixer presets
   */
  SimpleXMLTransfer* getMixerPresets(SimpleXMLTransfer* xml);
   
  /**
   * Set index of graphics to use.
   * 
   * @author Jens Wilhelm Wulf
   */
  void SetGraphics(SimpleXMLTransfer* xml, int nIdx);
   
  /**
   * Set index of config to use.
   * 
   * @author Jens Wilhelm Wulf
   */
  void SetConfig(SimpleXMLTransfer* xml, int nIdx);

  /**
   * Returns an TSimInputs::eSteeringMap corresponding to the string smstr.
   */
  TSimInputs::eSteeringMap GetSteering(std::string smstr);
};
#endif

