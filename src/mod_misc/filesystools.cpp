/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2004-2006, 2008, 2010 Jens Wilhelm Wulf (original author)
 * Copyright (C) 2005, 2006, 2007, 2008 Jan Reucker
 * Copyright (C) 2005 Lionel Cailler
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

#include "filesystools.h"

#include <fstream>
#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <errno.h>

#ifdef linux
# include <stdlib.h>     // getenv()
# include <sys/stat.h>   // mkdir
# include <sys/types.h>  // mkdir
#endif

#if defined(__APPLE__) || defined(MACOSX)
# include <stdlib.h>     // getenv()
# include <sys/stat.h>   // mkdir
# include <sys/types.h>  // mkdir
#endif

#ifdef WIN32
# define WIN32_LEAN_AND_MEAN
# include <windows.h>  // MoveFileEx
# include <stdlib.h>   // getenv()
# include <direct.h>   // mkdir
#endif

#include "SimpleXMLTransfer.h"

std::string FileSysTools::appname;

// see header
void FileSysTools::makeSurePathExists(std::string path)
{
  if (path.length() == 0)
    return;
  
  unsigned int   pos0 = 0;
  
  if (path[pos0] == '/')
    pos0++;
  
  // last character should be '/'
  if (path[path.length()-1] != '/')
    path.append("/");
  
  while (pos0 < path.length())
  {
    while (path[pos0] != '/')
      pos0++;

#if linux
    mkdir(path.substr(0, pos0).c_str(), 0766);
#endif
#if defined(__APPLE__) || defined(MACOSX) 
    mkdir(path.substr(0, pos0).c_str(), 0700);
#endif
#ifdef WIN32
    mkdir(path.substr(0, pos0).c_str());
#endif
    
    pos0++;
  }
}

// see header
bool FileSysTools::fileExists(std::string path)
{
  std::ifstream testDat;
  
  testDat.open(path.c_str());
  
  if (!testDat) 
    return(false);
  else 
  { 
    testDat.close();
    return(true);
  }  
}

// see header
void FileSysTools::getSearchPathList(std::vector<std::string>& pathlist, std::string dirname)
{
  // Search in:
  // current working directory (.)        (all)
  // $HOME/Documents/crrcsim              (APPLE/MACOSX)
  // /Library/Application Support/crrcsim (APPLE/MACOSX)
  // $USERPROFILE/.crrcsim                (WIN32)
  // $HOME/.crrcsim                       (LINUX)
  // CRRC_DATA_PATH                       (LINUX)
  // /usr/local/share/games/crrcsim       (LINUX)
  // /usr/share/games/crrcsim             (LINUX)

  // cwd
  if (dirname != "")
  {
    pathlist.push_back(dirname);
  }
  else
  {
    pathlist.push_back(".");
  }
  
  // home directories
  std::string homepath_str = getHomePath();
  if (homepath_str != "")
  {
    if (dirname != "")
    {
      homepath_str += '/';
      homepath_str.append(dirname);
    }
    pathlist.push_back(homepath_str);
  }

  // System-wide installation paths (Linux and Mac OS X only)
  #if defined(linux)
  std::string data_path;
  #ifdef CRRC_DATA_PATH
  data_path = CRRC_DATA_PATH;
  #endif

  if (dirname != "")
  {
    std::string s;
    data_path += '/';
    #ifdef CRRC_DATA_PATH
    s = data_path;
    s.append(dirname);
    pathlist.push_back(s);
    #endif
    s = "/usr/local/share/games/" + appname + "/";
    if (s != data_path)    // avoid adding this path twice
    {
      s.append(dirname);
      pathlist.push_back(s);
    }
    s = "/usr/share/games/" + appname + "/";
    if (s != data_path)    // avoid adding this path twice
    {
      s.append(dirname);
      pathlist.push_back(s);
    }
  }
  else
  {
    #ifdef CRRC_DATA_PATH
    pathlist.push_back(data_path);
    #endif
    if (data_path != "/usr/local/share/games/" + appname)  // avoid adding this path twice
    {
      pathlist.push_back("/usr/local/share/games/" + appname);
    }
    if (data_path != "/usr/share/games/" + appname)  // avoid adding this path twice
    {
      pathlist.push_back("/usr/share/games/" + appname);
    }
  }
  #endif
  #if defined(__APPLE__) || defined(MACOSX)
  {
    std::string s = "/Library/Application Support/" + appname;
    if (dirname != "")
    {
      s.append("/");
      s.append(dirname);
    }
    pathlist.push_back(s);
  }
  #endif
  #if 0
  std::cout << "T_Config::getSearchPathList():" << std::endl;
  for (std::vector<std::string>::size_type i = 0; i < pathlist.size(); i++)
  {
    std::cout << pathlist[i] << std::endl;
  }
  #endif
}

// see header
std::string FileSysTools::getHomePath()
{
  std::string homepath_str = "";
  char        *homepath = NULL;
  
#ifdef WIN32
  homepath = getenv("USERPROFILE");
#else
  homepath = getenv("HOME");
#endif

  if (homepath != NULL)
  {
    homepath_str  = homepath;
    #if defined(__APPLE__) || defined(MACOSX)
    homepath_str.append("/Documents/" + appname);
    #else
    homepath_str.append("/." + appname);
    #endif
  }
  
  return homepath_str;
}

// see header
std::string FileSysTools::getDataPath(std::string item, bool fThrowEx)
{
  std::string path = "";
  
  std::vector<std::string> possible_paths;
  FileSysTools::getSearchPathList(possible_paths, item);

  //~ std::cout << "T_Config::getDataPath(): " << std::endl;
  //~ for (std::vector<std::string>::size_type i = 0; i < possible_paths.size(); i++)
  //~ {
    //~ std::cout << possible_paths[i] << std::endl;
  //~ }

  for (std::vector<std::string>::size_type i = 0; i < possible_paths.size(); i++)
  {
    if (FileSysTools::fileExists(possible_paths[i]))
    {
      path = possible_paths[i];
      break;
    }
  }
  if ((path == "") && (fThrowEx))
  {
    // well..maybe another type of exception might be better here...
    throw XMLException("Unable to find a matching file: " + item);
  }

  return(path);
}

std::string FileSysTools::name(std::string pathAndName)
{
  std::string::size_type pos = pathAndName.rfind('/');
  
  if (pos != std::string::npos)
  {
    return(pathAndName.substr(pos+1));
  }
  else
    return(pathAndName);
}

int FileSysTools::move(std::string dest, std::string src)
{
#ifdef WIN32
  BOOL ret = MoveFileEx(src.c_str(), dest.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_COPY_ALLOWED );
  if (ret)
    return(0);
  else
    return(-1);
#else   
  if (dest[dest.length()-1] == '/')
    dest += name(src);
  
  std::string call  = "mv \"";
  call += src;
  call += "\" \"" + dest + "\"";
  
  return(
         system(call.c_str())
         );
#endif
}
