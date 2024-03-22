/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005 Jan Reucker (original author)
 * Copyright (C) 2005, 2006, 2008 Jens Wilhelm Wulf
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
  

// puaFileBox.cpp - Implementation of puaFileBox

#include "puaFileBox.h"
#include <sys/types.h>
#include <dirent.h>
#include <iostream>
#include <algorithm>

/** \brief Create a puFileBox by specifying size, path and extension.
 *
 *  \param minx xpos of lower left corner
 *  \param miny ypos of lower left corner
 *  \param maxx xpos of upper right corner
 *  \param maxy ypos of upper right corner
 *  \param path Path to the directory to be searched in.
 *  \param extension The extensions of the files you want to add (comma separated).
 *  \param case_insensitive Perform a case insensitive search? Default is false (case sensitive).
 */
puaFileBox::puaFileBox( int minx, int miny, int maxx, int maxy,
                        const char *path,
                        const char *extension,
                        bool case_insensitive)
                        :puaScrListBox(minx, miny, maxx, maxy),
                         entries(NULL)
{
  addItems(path, extension);
}


/**
 *  Destroy the widget.
 */
puaFileBox::~puaFileBox()
{
  std::vector<std::string *>::iterator it;
  
  // destroy all entries of the files vector
  for (it = files.begin(); it != files.end(); it++)
  {
    delete *it;
  }
  files.clear();
  
  // destroy all entries of the paths vector
  for (it = paths.begin(); it != paths.end(); it++)
  {
    delete *it;
  }
  paths.clear();
  
  delete[] entries;
  entries = NULL;
}


/** \brief Test if a file name has a specific extension
 *
 *  \param fname A string containing the file name to check.
 *  \param ext A string containing the extension.
 *  \param case_insensitive Perform a case insensitive search? Defaults to false (case sensitive).
 *  \return True if the file name has the given extension, false otherwise.
 */
bool puaFileBox::checkExtension(std::string fname, std::string ext, bool case_insensitive)
{
  bool result = false;
  std::string::size_type dot_index;
  std::string file_ext;

  if (case_insensitive)
  {
    std::transform(ext.begin(), ext.end(), ext.begin(), toupper);
    std::transform(fname.begin(), fname.end(), fname.begin(), toupper);
  }
#ifdef DEBUG_FILE_BOX
  std::cout << "Searching for " << ext << " in " << fname << std::endl;
#endif
  dot_index = fname.find_last_of('.');

  // Error handling: file name did not contain a dot
  if (dot_index == std::string::npos)
  {
#ifdef DEBUG_FILE_BOX
    std::cout << "(no dot) no match";
#endif
  }
  // Error handling: no extension (dot was last character)
  else if (dot_index == (fname.length() - 1))
  {
#ifdef DEBUG_FILE_BOX
    std::cout << "(no ext) no match";
#endif
  }
  // found beginning of extension, now compare it to ext
  else
  {
    file_ext = fname.substr(dot_index+1);
    if (file_ext == ext)
    {
      result = true;
#ifdef DEBUG_FILE_BOX
      std::cout << "MATCH!";
#endif
    }
    else
    {
#ifdef DEBUG_FILE_BOX
      std::cout << "no match";
#endif
    }
  }

#ifdef DEBUG_FILE_BOX
    std::cout << std::endl;
#endif

  return result;
}


/** \brief Add items with a given extension to the ListBox.
 *
 *  All file names with the given extension in the directory
 *  specified by <code>path</code> are added to the ListBox.
 *  By default, the comparison between file names and the
 *  extension is case sensitive.
 *  \param path directory to browse
 *  \param extension one or more file extensions of the files to be added to the widget (comma separated)
 *  \param case_insensitive Set to <code>true</code> for case insensitive browsing
 *  \return the number of newly added items
 */
int puaFileBox::addItems(const char *path, const char *extension, bool case_insensitive)
{
  DIR *dir;
  struct dirent *ent;
  int numElem, i;
  int newEntries  = 0;
  
  std::string              ext      = extension;
  std::vector<std::string> extlist;
  
  {
    std::string::size_type start = 0;
    std::string::size_type pos;
    
    while ( (pos = ext.find(',', start)) != std::string::npos)
    {
      extlist.push_back(ext.substr(0, pos));
      start = pos+1;
    }
    extlist.push_back(ext.substr(start));
  }
  

  // Show path. This is not how we wanted to do it, but this is a 
  // fix at least.
  {
    std::string* dirinfo;
    dirinfo = new std::string();
    
    dirinfo->append("--- ");
    dirinfo->append(path);
    files.push_back(dirinfo);
    paths.push_back((std::string*)0);
  }
  
  if ((dir = opendir(path)) == NULL)
  {
    std::cerr << "puaFileBox::addItems(): unable to open directory " << path;
    std::cerr << std::endl; 
  }
  else
  {
    while ((ent = readdir(dir)) != NULL)
    {
      std::string *tmp   = new std::string();
      bool        fMatch = false;
      
      *tmp = ent->d_name;
      
      for (unsigned int n=0; n<extlist.size() && fMatch == false; n++)
      {
        if (checkExtension(*tmp, extlist[n].c_str(), case_insensitive))
          fMatch = true;
      }
                 
      if (fMatch)
      {
        //std::cout << *tmp << std::endl;
        std::string *thePath = new std::string(path);
        files.push_back(tmp);
        paths.push_back(thePath);
        newEntries++;
      }
      else
      {
        delete tmp;
      }
    }
    closedir(dir);
  
    delete[] entries;
    numElem = files.size();
    entries = new char*[numElem+1];
    
    for (i = 0; i < numElem; i++)
    {
      entries[i] = (char*)files[i]->c_str();
      //std::cout << "+ " << entries[i] << std::endl;
    }
    entries[numElem] = NULL;
    newList(entries);
  }
  
  return newEntries;
}


/** \brief Get the current item's full path.
 *
 *  The FileBox only displays the name of a file, not the
 *  full path. To access the file, the path to each item
 *  can be queried with this method. Call it without a
 *  parameter to get the currently selected item's path,
 *  or supply the index of the element of which you want
 *  to get the path name.
 *
 *  The path does not include the file name. Retrieve the
 *  current item's file name by using the widget's
 *  getStringValue() method.
 *
 *  For compatibility reasons with the rest of PUI, the
 *  path is returned as <code>const char*</code> although a
 *  std::string would be more convenient.
 *
 *  \param index Index of a list item
 *  \return Path to the item or NULL if no item is selected or
 *          the item specified by <code>index</code> does not exist.
 */
const char * puaFileBox::getPath(int index)
{
  const char *ret = NULL;

  if (index == -1)
  {
    // get currently highlighted item's path
    index = getIntegerValue();
  }
  
  if ((index >= 0) && (index < getNumItems()))
  {
    // get path of item "index"
    if (paths[index] == (std::string*)0)
      ret = (const char*)0;
    else
      ret = (*(paths[index])).c_str();
  }

  return ret;
}

