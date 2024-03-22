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
  

/** \file crrc_ssgutils.cpp
 *
 *  Helper functions for working with PLIB SSG models.
 */

#include "crrc_ssgutils.h"


/**
 *  \brief Locate a named SSG node in a branch.
 *
 *  This method locates a named node in an SSG scenegraph branch. The
 *  node name usually corresponds to the name of an object in the 3D model.
 *
 *  This method was taken from SimGear, credits go to the original
 *  authors.
 *
 *  \param  node    Pointer to the scenegraph branch
 *  \param  name    Name of the object to search for
 *  \return Pointer to the node or NULL if no node with this name was found
 */
ssgEntity* SSGUtil::findNamedNode (ssgEntity* node, const char* name)
{
  std::string NodeName = SSGUtil::getPureNodeName(node);
  if (NodeName == name)
  {
    return node;
  }
  else if (node->isAKindOf(ssgTypeBranch()))
  {
    int nKids = node->getNumKids();
    for (int i = 0; i < nKids; i++)
    {
      ssgEntity * result =
        findNamedNode(((ssgBranch*)node)->getKid(i), name);
      if (result != 0)
      {
        return result;
      }
    }
  } 
  return NULL;
}


/**
 *  Splice a branch in between all child nodes and their parents.
 *
 *  Inserts the given branch in the scenegraph above the
 *  entity \c child.
 *
 *  This method was taken from SimGear, credits go to the original
 *  authors.
 *
 *  \param  branch    the branch to be inserted
 *  \param  child     the location where the branch shall be inserted
 */
void SSGUtil::spliceBranch (ssgBranch * branch, ssgEntity * child)
{
  int nParents = child->getNumParents();
  branch->addKid(child);
  for (int i = 0; i < nParents; i++)
  {
    ssgBranch * parent = child->getParent(i);
    parent->replaceKid(child, branch);
  }
}


/** Get the "pure" name of a node (without any additional attributes).
 *
 *  A 3D model file usually contains several independent objects.
 *  Each object appears as a distinct node in the scenegraph. Most
 *  modelling tools allow to assign an arbitrary name to an object.
 *  CRRCsim decomposes this string into two parts: everything up
 *  to the first blank character is interpreted as the real name
 *  of a node, while the rest of the string is parsed for some
 *  known attributes that may describe the node's appearance or
 *  behaviour.
 *
 *  This method extracts the first part (the pure name) from a node.
 *
 *  \param  node  pointer to a node in the scenegraph
 *  \return first part of the object's name string up to first blank
 *  
 */
std::string SSGUtil::getPureNodeName(ssgEntity* node)
{
  std::string NodeName;
  if (node->getName() != NULL)
  {
    NodeName = node->getName();
  }

  std::string::size_type spaceposn = NodeName.find(' ');

  if (spaceposn == std::string::npos)
  {
    // node name does not contain any additional attribute
    return NodeName;
  }
  else
  {
    // node name contains attributes, extract everything up to first blank
    return NodeName.substr(0, spaceposn);
  }
}


/** 
 * A string tokenizer.
 *
 * This generic string tokenizer was taken from the GNU libstdc++
 * documentation.
 * http://gcc.gnu.org/onlinedocs/libstdc++/manual/bk01pt05ch13s04.html
 *
 * \param container   Reference to a container that stores the tokens.
 *                    This may be any variable of type std::list, std::vector
 *                    and so on
 * \param delimiters  List of token-delimiting characters
 */
template <typename Container>
void stringtok(Container &container, std::string const &in,
               const char * const delimiters = " \t\n")
{
  const std::string::size_type len = in.length();
  std::string::size_type i = 0;

  while (i < len)
  {
    // Eat leading whitespace
    i = in.find_first_not_of(delimiters, i);
    if (i == std::string::npos)
    {
      return;   // Nothing left but white space
    }
    
    // Find the end of the token
    std::string::size_type j = in.find_first_of(delimiters, i);

    // Push token
    if (j == std::string::npos) 
    {
      container.push_back(in.substr(i));
      return;
    } 
    else
    {
      container.push_back(in.substr(i, j-i));
    }
    
    // Set up for next loop
    i = j + 1;
  }
}



/** Get any additional attributes that may be appended to the node's name 
 *
 *  A 3D model file usually contains several independent objects.
 *  Each object appears as a distinct node in the scenegraph. Most
 *  modelling tools allow to assign an arbitrary name to an object.
 *  CRRCsim decomposes this string into two parts: everything up
 *  to the first blank character is interpreted as the real name
 *  of a node, while the rest of the string is parsed for some
 *  known attributes that may describe the node's appearance or
 *  behaviour.
 *
 *  This method extracts all known attributes from the object's name
 *  string.
 *
 *  \param  node  pointer to a node in the scenegraph
 *  \return struct with attribute values
 */
SSGUtil::NodeAttributes SSGUtil::getNodeAttributes(ssgEntity* node)
{
  std::string NodeName;
  SSGUtil::NodeAttributes attribs;

  if (node->getName() != NULL)
  {
     NodeName = node->getName();
  }
  
  std::string::size_type posn = NodeName.find(' ');

  if (posn != std::string::npos)
  {
    // node name contains attributes, only evaluate text after the blank
    NodeName = NodeName.substr(posn);

    // break remaining text into tokens
    stringtok(attribs.rawAttributes, NodeName, " ");
  }

  return attribs;
}


/** \brief Remove a leaf node from a scenegraph
 *
 *  Removes a leaf node from all of its parents in a scenegraph. This
 *  will usually cause the reference count for this leaf to drop to zero.
 *  In this case the leaf will be deleted automatically.
 *
 *  \param leaf The leaf to be removed.
 */
void SSGUtil::removeLeafFromGraph(ssgLeaf *leaf)
{
  if (leaf != NULL)
  {
    //std::cout << "  Removing node " << SSGUtil::getPureNodeName(leaf) 
    //          << " from graph." << std::endl;
    int num_parents = leaf->getNumParents();
    for (int i = 0; i < num_parents; i++)
    {
      leaf->getParent(i)->removeKid(leaf);
    }
  }
}

