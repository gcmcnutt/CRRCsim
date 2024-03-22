/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2004-2006, 2008, 2010 Jens Wilhelm Wulf (original author)
 * Copyright (C) 2005, 2006 Jan Reucker
 * Copyright (C) 2006 Todd Templeton
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
  
#ifndef __FILESYSTOOLS_H
#define __FILESYSTOOLS_H

#include <string>
#include <vector>

/**
 * Class for file(system) related methods which are not application-specific.
 */
class FileSysTools
{
public:
  
  /**
   * Utility function which makes sure some path exists -- parts of it are created,
   * if needed.
   */
  static void makeSurePathExists(std::string path);
  
  /**
   *  Get the full path to the given data item
   *
   * Finds most local path to a datafile, for example
   * "sounds/fan.wav", "models/allegro.air" or "textures/beachsand.rgb".
   * Search order depends on operating system.
   * This function should be used if one doesn't know where a data file
   * really is.
   * 
   *  Pass the filename and relative path to this function and it
   *  will search for a suitable file in the search path. The
   *  absolute path to this file, including the filename,
   *  will be returned.
   *
   *  \param item       data item to search for (filename and relative path)
   *  \param fThrowEx   if set to true, an exception will be thrown if no
   *                    matching file was found
   *
   *  \return absolute path to the file, empty string on error
   */
  static std::string getDataPath(std::string item, bool fThrowEx = false);
  
  /**
   * Provide a list of possible locations for data files or
   * other directories. If you provide a dirname, this string will
   * be appended to each path. The locations heavily depend on
   * the platform CRRCsim is running on.
   *
   * Example: getSearchPathList(list, "models") will fill "list"
   * with entries like "models", "/home/johndoe/.crrcsim/models",
   * "/usr/local/share/crrcsim/models" and so on.
   *
   * \param pathlist Reference to a list that will be filled with the pathnames
   * \param dirname  string to be appended to each path
   */
  static void getSearchPathList(std::vector<std::string>& pathlist,
                                std::string dirname = "");
  
  /**
   *  Test if a file exists.
   *
   *  \param  path   File name and path to test.
   *  \retval true   if file exists
   *  \retval false  if file does not exist
   */
  static bool fileExists(std::string path);
  
  /**
   * Get the path to the CRRCsim directory inside the
   * user's home directory. This usually is OS-dependent.
   */
  static std::string getHomePath();

  /**
   * Set application name
   * 
   * The application name will be used to form some of directory names used in this class.
   */
  static void SetAppname(std::string name) { appname = name; };
  
  /**
   * Returns name and suffix of a file, strips leading directory information.
   * 
   * @author Jens W. Wulf
   */
  static std::string name(std::string pathAndName);
  
  /**
   * Moves a file.
   * dest can be a file or a directory (ends with '/').
   * 
   * @author Jens W. Wulf
   */
  static int move(std::string dest, std::string src);
  
private:
  static std::string appname;
};

#endif
