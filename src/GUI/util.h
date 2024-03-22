/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2006, 2008 Jens Wilhelm Wulf (original author)
 * Copyright (C) 2006, 2008, 2009 Jan Reucker
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
  

#ifndef UTIL_H
#define UTIL_H

#include "../mod_misc/SimpleXMLTransfer.h"

class T_GUI_Util
{
  public:
   
   /**
    * Loads a list of names from <code>grp</code> into an array,
    * memory is allocated as needed.
    * If fDefault then there always is the first entry 'default'. 
    * The last item in the list is a (char*)0.
    * nSize returns the number of entries (excluding (char*)0).
    */
   static char** loadnames(SimpleXMLTransfer* grp,
                           int&               nSize,
                           bool               fDefault = true);

   /**
    * Creates an array of names from the vector of strings,
    * memory is allocated as needed. The last item in the
    * list is a NULL-Pointer.
    * \param list list of names as vector of std::strings
    * \param nSize returns the number of entries (excluding NULL entry)
    * \return pointer to char[] containing the names
    */
   static char** loadnames( std::vector<std::string>& list,
                            int&                      nSize);

   /**
    * Frees memory allocated by the above methods.
    */
   static void   freenames(char**& ptr,
                           int     nSize);
  
  /**
   * Get index of a name from a list of names created by T_GUI_Util::loadnames()
   *
   * \param   list    List of names created by T_GUI_Util::loadnames()
   * \param   name    Name of which the index shall be retrieved
   * \return  Index of the element "name" (first element = 0) or -1 if not found
   */
   static int    findname(char**& list, std::string name);

   /// Compare a file extension to a given string
   static bool   checkExtension(std::string fname,
                                std::string ext,
                                bool case_insensitive = false);

    /** \brief Remove excessive whitespace from a string
     *
     *  This method removes any excessive whitespace from a string:
     *
     *  - leading whitespace characters
     *  - trailing whitespace characters
     *  - multiple successive whitespace characters
     *
     *  In addition, all remaining whitespace characters will be
     *  converted to plain spaces (' ').
     *
     *  \param  str The string to be trimmed.
     *  \return     String without excessive whitespace
     */
    static std::string trimWhitespace(std::string str);

    /** \brief Insert line-breaks into a string
     *
     *  This method inserts newline characters ('\n') into a string
     *  to create lines of at most line_length characters.
     *  If one single word on a line exceeds the maximum line_length,
     *  the newline character will be placed right behind this word.
     *  In this case the line will have more that line_length characters.
     *
     *  \param  str         input string
     *  \param  line_length desired line length
     *  \return string broken into lines of at most line_length characters
     */
    static std::string breakLines(std::string str, unsigned int line_length);


    /** \brief clean string before set on puaLargeInput
    *
    **/
    static std::string cleanText(std::string description_string);
    
};
#endif
