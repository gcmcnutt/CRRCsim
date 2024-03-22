/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2006, 2008 Jens Wilhelm Wulf (original author)
 * Copyright (C) 2006, 2008,2009 Jan Reucker
 * Copyright (C) 2006 Lionel Cailler
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
  

#include "util.h"

#include <cstring>
#include <string>
#include <cstdlib>
#include <algorithm>

char** T_GUI_Util::loadnames(SimpleXMLTransfer* grp,
                             int&               nSize,
                             bool               fDefault)
{
  char**              presets;
  SimpleXMLTransfer*  preset;
  std::string         name;
  int                 index;
  int                 nDefault = 0;

  if (grp != (SimpleXMLTransfer*)0)
    nSize = grp->getChildCount();
  else
    nSize = 0;
  
  if (fDefault)
  {
    nDefault = 1;
  }
  
  presets = (char**)malloc(sizeof(char**) * (nSize+nDefault+1));

  if (fDefault)
  {
    name = "default";
    presets[0] = (char*)malloc(sizeof(char**) * (name.length()+1));
    strcpy(presets[0], name.c_str());
  }
  
  for (int n=nDefault; n<nSize+nDefault; n++)
  {
    preset = grp->getChildAt(n-nDefault);
    index  = preset->indexOfAttribute("name_en");
    if (index < 0)
      name = preset->getContentString();
    else
      name   = preset->attribute("name_en");
    presets[n] = (char*)malloc(sizeof(char**) * (name.length()+1)); // shouldn't it be sizeof(char)?
    strcpy(presets[n], name.c_str());
  }      
  
  presets[nSize+nDefault] = (char*)0;
  
  return(presets);
}


char** T_GUI_Util::loadnames( std::vector<std::string>& list,
                              int&                      nSize)
{
  char **names;
  
  names = (char**)malloc(sizeof(char**) * (list.size() + 1));
  
  for (unsigned int i = 0; i < list.size(); i++)
  {
    names[i] = (char*)malloc(sizeof(char) * (list[i].length() + 1));
    strcpy(names[i], list[i].c_str());
  }
  names[list.size()] = NULL;
  nSize = list.size();
  
  return names;
}


void  T_GUI_Util::freenames(char**& ptr,
                            int     nSize)
{
  if (ptr != NULL)
  {
    for (int n=0; n<nSize; n++)
    {
      free(ptr[n]);
    }
    free(ptr);
  }
}


int T_GUI_Util::findname(char**& list, std::string name)
{
  int nIndex = -1;
  
  for (int i = 0; list[i] != NULL; ++i)
  {
    if (name.compare(list[i]) == 0)
    {
      nIndex = i;
    }
  }
  return nIndex;
}

/** \brief Test if a file name has a specific extension
 *
 *  \param fname A string containing the file name to check.
 *  \param ext A string containing the extension.
 *  \param case_insensitive Perform a case insensitive search? Defaults to false (case sensitive).
 *  \return True if the file name has the given extension, false otherwise.
 */
bool T_GUI_Util::checkExtension(std::string fname, std::string ext, bool case_insensitive)
{
  bool result = false;
  std::string::size_type dot_index;
  std::string file_ext;

  if (case_insensitive)
  {
    std::transform(ext.begin(), ext.end(), ext.begin(), toupper);
    std::transform(fname.begin(), fname.end(), fname.begin(), toupper);
  }
#ifdef DEBUG_GUI_UTIL
  std::cout << "Searching for " << ext << " in " << fname << std::endl;
#endif
  dot_index = fname.find_last_of('.');

  // Error handling: file name did not contain a dot
  if (dot_index == std::string::npos)
  {
#ifdef DEBUG_GUI_UTIL
    std::cout << "(no dot) no match";
#endif
  }
  // Error handling: no extension (dot was last character)
  else if (dot_index == (fname.length() - 1))
  {
#ifdef DEBUG_GUI_UTIL
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
#ifdef DEBUG_GUI_UTIL
      std::cout << "MATCH!";
#endif
    }
    else
    {
#ifdef DEBUG_GUI_UTIL
      std::cout << "no match";
#endif
    }
  }

#ifdef DEBUG_GUI_UTIL
    std::cout << std::endl;
#endif

  return result;
}

  
std::string T_GUI_Util::trimWhitespace(std::string str)
{
  bool last_was_white = true;
  bool pending_white = false;
  std::string clean_copy;
  std::string::iterator it;
  
  for (it = str.begin(); it != str.end(); it++)
  {
    if (isspace(*it))
    {
      if (!last_was_white)
      {
        last_was_white = true;
        pending_white = true;
      }
    }
    else
    {
      last_was_white = false;
      if (pending_white)
      {
        clean_copy += ' ';
        pending_white = false;
      }
      clean_copy += *it;
    }
  }
  return clean_copy;
}


std::string T_GUI_Util::breakLines(std::string str, unsigned int line_length)
{
  const char *whitespace = " \f\n\r\t\v";
  std::string::iterator it;
  std::string::size_type pos;
  std::string out;
  
  while (!str.empty())
  {
    if (str.length() <= line_length)
    {
      // rest of string fits in the line
      out += str;
      str.erase();
    }
    else
    {
      pos = str.find_last_of(whitespace, line_length);
      if (pos == std::string::npos)
      {
        // no whitespace found within the first line_length characters,
        // search for the first whitespace in the string
        pos = str.find_first_of(whitespace);
        if (pos == std::string::npos)
        {
          // no whitespace at all, copy whole string
          out += str;
          str.erase();
        }
        else
        {
          // copy from first character up to the whitespace
          // (whitespace itself won't be copied)
          out += std::string(str, 0, pos);
          out += '\n';
          str.erase(0, pos + 1);  // +1 to eliminate the whitespace
        }
      }
      else
      {
        // whitespace found at position pos
        out += std::string(str, 0, pos);
        out += '\n';
        str.erase(0, pos + 1);  // +1 to eliminate next whitespace, too
      }
    }
  }
  return out;
}


/***** clean string before set on puaLargeInput
*       used now by crrc_location.cpp & crrcs_planesel.cpp ***************/
std::string T_GUI_Util::cleanText(std::string description_string)
{
/* history :
  1)  description_string = T_GUI_Util::trimWhitespace(description_string);
      description_string = T_GUI_Util::breakLines(description_string, 54);
      ==>  Loss of line feeds wanted by the writer.
          The function 'T_GUI_Util::breakLines' is imperfect because it does not take into account the width of the characters

  2) "Don't use any black magic when reformatting the description string. One would 
       have to tell people about how it worked and it would be limiting or lots of 
       code...so just trim a leading empty line."
       ==> Certain existing descriptions have long lines who obliges to use of the horizontal scroll bar. It is impractical.
       
  3) We put the cut of lines, but by parameter "wrap=true" to the call of puaLargeInput (in crrc_location.cpp & crrcs_planesel.cpp). It works much better.
      ==> TODO ? Use an writing convention to prevent that the end of line in the file xml is considered as the end of paragraph. Example: "\\" at the end of line
*/
    //just trim a leading empty line
    std::string::size_type pos = 0;
    char c;
    while (pos < description_string.length() && isspace(description_string[pos]))
      pos++;
    description_string = description_string.substr(pos);
    // suppress carriage return (CR) for file edited on MSWindows
    std::string description_string0 = description_string;
    description_string.clear();
    pos = 0;
    while (pos < description_string0.length())
    {
      c = description_string0[pos];
      if( c != '\r') description_string.push_back (c);
      pos++;
    }
    return description_string;
}
