/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005 Jan Reucker (original author)
 * Copyright (C) 2005, 2008 Jens Wilhelm Wulf
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
  

// puaFileBox.h - A list of file names.

#ifndef PUAFILEBOX_H
#define PUAFILEBOX_H

#include "puaScrListBox.h"
#include <vector>
#include <string>

/** \brief A scrollable list box filled with file names.
 *
 *  This class represents a scrollable PUI list box which
 *  is filled with file names. By specifying a file extension
 *  and a directory it's easy to automatically list a
 *  specific group of files.
 */
class puaFileBox : public puaScrListBox
{
  public:
    puaFileBox( int x, int y, int w, int h,
                const char *path, const char *extension,
                bool case_insensitive = false);
    virtual ~puaFileBox();
  
    virtual int addItems( const char *path, 
                          const char *extension,
                          bool case_insensitive = false);
    virtual const char * getPath(int index = -1);

  private:
    bool checkExtension(std::string fname,
                        std::string ext,
                        bool case_insensitive = false);
    std::vector<std::string *> files;
    std::vector<std::string *> paths;
    char          **entries;
};



#endif  // PUAFILEBOX_H
