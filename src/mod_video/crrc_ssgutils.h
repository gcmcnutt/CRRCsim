/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2008 Jan Reucker (original author)
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
  

/** \file crrc_ssgutils.h
 *
 *  Utility functions for working with PLIB SSG models.
 */

#ifndef CRRC_SSGUTILS_H_
#define CRRC_SSGUTILS_H_

#include <plib/ssg.h>
#include <string>
#include <list>

namespace SSGUtil
{


/**
 *  This class stores the attributes that may be added to a
 *  node of a 3D object by appending them to the node's name.
 *  E.g. a node with the name "rotor -shadow" is interpreted by
 *  CRRCsim as a node with the name "rotor" and the attribute
 *  "shadow" set to "false".
 */
class NodeAttributes
{
  friend NodeAttributes getNodeAttributes(ssgEntity* node);
  
  public:
    NodeAttributes()
    : rawAttributes()
    {
    }
  
    
    /**
     * Check if the given attribute is specified for the node.
     *
     * This method will return 0, 1 or -1 if the given attribute
     * is not specified, specified positively or specified negatively.
     *
     * \param attribute     Name of an attribute (e.g. "shadow")
     *
     * \retval  0   Attribute is not specified for the node
     * \retval  1   Attribute is specified positively (e.g. "+shadow")
     * \retval -1   Attribute is specified negatively (e.g. "-shadow")
     */
    int checkAttribute(std::string attribute) const
    {
      int retval = 0;
      std::string pos_attr = std::string("+") + attribute;
      std::string neg_attr = std::string("-") + attribute;
      
      for (std::list<std::string>::const_iterator it = rawAttributes.begin();
           it != rawAttributes.end();
           it++)
      {
        if (*it == pos_attr)
        {
          retval = 1;
          break;
        }
        else if (*it == neg_attr)
        {
          retval = -1;
          break;
        }
      }
      
      return retval;
    }

  private:
    std::list<std::string> rawAttributes;
};
  
/** Locate a named SSG node in a branch. */
ssgEntity* findNamedNode (ssgEntity* node, const char* name);

/** Splice a branch in between all child nodes and their parents. */
void spliceBranch (ssgBranch * branch, ssgEntity * child);

/** Get the "pure" name of a node (without any additional attributes). */
std::string getPureNodeName(ssgEntity* node);

/** Get any additional attributes that may be appended to the node's name */
SSGUtil::NodeAttributes getNodeAttributes(ssgEntity* node);

/** Remove a leaf node from a scenegraph */
void removeLeafFromGraph(ssgLeaf *leaf);

} // end namespace

#endif  // CRRC_SSGUTILS_H_

