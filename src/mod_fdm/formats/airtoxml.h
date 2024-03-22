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
#ifndef AIRTOXML_H
#define AIRTOXML_H

/**
 * This module converts air-files to the new xml-representation.
 * With the exception of the power and propulsion system, things 
 * are mostly copied in a structured way.
 *
 * Most sections are given a version number to make future changes 
 * easier.
 * 
 * Furthermore the old-style sections containing numerical parameters 
 * get an attribute 'units' which is set to zero to show that these are
 * the numbers from the .air file. This makes it possible to optionally 
 * use SI units (or something else) in those sections instead.
 * 
 * This attribute is set to one in the power section, as it does use
 * SI units.
 *
 * @author Jens Wilhelm Wulf
 */


#include <string>


/**
 * Returns file to load.
 * Depending on config, it tries to convert file.
 */
std::string air_to_xml_file_load(std::string filename);

/**
 * filename: returns xml instead of air
 */
std::string getXMLFilename(std::string filename);

/**
 * Converts a single file.
 * Returns number of files successfully converted.
 * Does also return 1 if there already was an xml file.
 */
int air_to_xml_file(std::string filename);


/**
 * Automatic conversion of all files which can be found.
 * Returns number of files successfully converted.
 */
int air_to_xml();

#endif
