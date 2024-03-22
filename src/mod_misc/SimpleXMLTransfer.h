/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2003, 2004, 2005, 2006, 2008 Jens Wilhelm Wulf (original author)
 * Copyright (C) 2004, 2008 Jan Reucker
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

/**
 * \file SimpleXMLTransfer.h
 *
 * Copyright (C) 2003-2006  Jens Wilhelm Wulf, j dot w dot wulf at gmx dot net
 * Licensed under the GNU General Public License version 2.
 */

#ifndef _SIMPLEXMLTRANSFER_H_
#define _SIMPLEXMLTRANSFER_H_

#include <string>
#include <vector>
#include <fstream>

typedef enum
{
  STATE_IDLE,
  STATE_START,
  STATE_INITTAG,
  STATE_INPUT,
  STATE_READNAME,
  STATE_WAIT_CONT,
  STATE_CONT_START,
  STATE_ENDTAG,
  STATE_ATTR_NAME,
  STATE_ATTR_VAL,
  STATE_ATTR_VAL_START,
  STATE_COMMENT,
  STATE_ATTR_VAL_WAIT_EQ
} T_State;


/** \brief Standard exception for SimpleXMLTransfer.
 * 
 *  This is the standard exception which is thrown by many methods of
 *  SimpleXMLTransfer if an operation fails.
 */
class XMLException
{
  public:
    /**
     * Create a new XMLException with a given <code>message</code>.
     */
    XMLException(std::string message);

    /**
     * Return a <code>const char*</code> to the exception's message.
     */
    const char* what();
  private:
    std::string myMessage;
};


class SimpleXMLTransfer;

/** \brief Simple XML parser class.
 *
 * This class should correspond to the Java-class jCoCo.SimpleXMLTransfer.
 * 
 * I started the implementation on November 12, 2003. Not all functionality
 * was included.
 * 
 * The method 'removeChildAt(int)' is not present in the Java-class and was
 * only implemented here.
 */
class SimpleXMLTransfer
{
  public:

    /**
     * Create an empty instance.
     */
    SimpleXMLTransfer();

    /**
     * Read from a file.
     * \param source Path to the file.
     */
    SimpleXMLTransfer(std::string source);

    /**
     * Read from a stream.
     */
    SimpleXMLTransfer(std::istream& in);

    /**
     * Copy
     */
    SimpleXMLTransfer(SimpleXMLTransfer* source);

    ~SimpleXMLTransfer();

    /**
     * Returns a string describing the source of this xml data
     */
    std::string getSourceDescr() const { return(sourcedescr); };
   
    /**
     * Return the name of the object.
     */
    std::string getName();

    /**
     * Print the element's data to stdout.
     */
    void print();

    /**
     * Print the element's data to a stream.
     * \param out Output stream.
     */
    void print(std::ostream& out);
   
    /**
     * Print data without header
     */
    void print(std::ostream& out,
               int           nIndent);   

    /**
     * Return the number of children.
     */
    int getChildCount();

    /**
     * Return the child with the index <code>childIndex</code>.
     */
    SimpleXMLTransfer* getChildAt(int childIndex);

    /**
     * Delete attributes. If an attribute does not exist an exception
     * will be thrown.
     */
    void delAttribute(std::string attribute);

    /**
     * Returns the integer value of the attribute <code>attr</code> or
     * throws an XMLException if the attribute does not exist.
     */
    int attributeAsInt(std::string attr);

    /**
     * Returns the integer value of the attribute <code>attr</code>.
     * If the attribute does not exist the function will return the
     * default value <code>defaultVal</code>.
     */
    int attributeAsInt(std::string attr,
                       int         defaultVal);

    /**
     * Returns the floating point value of the attribute <code>attr</code> or
     * throws an XMLException if the attribute does not exist.
     */
    float attributeAsFloat(std::string attr);

    /**
     * Returns the value of the attribute <code>attr</code> as a string or
     * throws an XMLException if the attribute does not exist.
     */
    std::string attribute(std::string attr);

    /**
     * Returns the first child named <code>child</code> or throws
     * an <code>XMLException</code> if no child with this name exists
     * and <code>fCreate == false</code>. If the latter is <code>true</code>,
     * the child is created.
     * 
     * It's legal to access children's children, e.g.
     * <code>getChild("eat.breakfast.bread")</code>
     * instead of
     * <code>getChild("eat").getChild("breakfast").getChild("bread")</code>
     */
    SimpleXMLTransfer* getChild(std::string child, bool fCreate = false);

    /**
     * Removes the child at position <code>nIndex</code> from the list of
     * children. The child's memory will not be freed!
     */
    void removeChildAt(int nIndex);

    /**
     * Removes the child <code>XMLPtr</code> from the list of
     * children. The child's memory will not be freed!
     */
     void removeChild(SimpleXMLTransfer* XMLPtr);
   
   /**
    * Replaces the child XMLPtrOld with XMLPtrNew.
    * The memory of XMLPtrOld will not be freed!
    */
   void replaceChild(SimpleXMLTransfer* XMLPtrOld,
                     SimpleXMLTransfer* XMLPtrNew);
   
   /**
     * Returns the content as a std::string.
     */
    std::string getContentString();

    /**
     * Set the content.
     */
    void setContent(std::string newContent);

    /**
     * Returns the value of the attribute <code>attr</code> as a std::string.
     * If the attribute does not exist the function will return the
     * default value <code>defaultVal</code>.
     */
    std::string attribute(std::string attr,
                          std::string defaultVal);

    /**
     * Returns the double precision floating point value of the attribute
     * <code>attr</code>.
     * If the attribute does not exist the function will return the
     * default value <code>defaultVal</code>.
     */
    double attributeAsDouble(std::string  attr,
                             double       defaultVal);

    /**
     * Returns the double precicion floating point value of the attribute
     * <code>attr</code> or throws an XMLException if the attribute does not exist.
     */
    double attributeAsDouble(std::string attr);

    /**
     * Returns the double precicion floating point value of the attribute
     * or throws an XMLException if the attribute does not exist.
     */
    double attributeAsDouble(unsigned int index);

    /**
     * Return the content as a double precision floating point value.
     */
    double getContentDouble();

    /**
     * Return the content as an integer value.
     */
    int getContentInt();

    /**
     * Quick access function to an attribute of the element or of a
     * child's child.
     * Throws exception if the attribute can't be found.
     */
    std::string getString(std::string path);

    /**
     * Quick access function to an attribute of the element or of a
     * child's child.
     * Throws exception if the attribute can't be found.
     * 
     * Possible notation for integer-values: decimal, hexadecimal with 
     * prefixed '0x' or binary with prefixed '0b'.
     */
    int getInt(std::string path);

    /**
     * Quick access function to an attribute of the element or of a
     * child's child.
     * Throws exception if the attribute can't be found.
     */
    double getDouble(std::string path);

    /**
     * Quick access function to an attribute of the element or of a
     * child's child.
     * Creates attribute with and returns default value if there
     * is no such attribute.
     */
    std::string getString(std::string path, std::string stringDefault);

    /**
     * Quick access function to an attribute of the element or of a
     * child's child.
     * Creates attribute with and returns default value if there
     * is no such attribute.
     * 
     * Possible notation for integer-values: decimal, hexadecimal with 
     * prefixed '0x' or binary with prefixed '0b'.
     */
    int getInt(std::string path, int nDefault);

    /**
     * Quick access function to an attribute of the element or of a
     * child's child.
     * Creates attribute with and returns default value if there
     * is no such attribute.
     */
    double getDouble(std::string path, double dDefault);

#if 1 == 2

    /**
     * Returns the single precision floating point value of
     * the attribute <code>attr</code>.
     * If the attribute does not exist the function will return the
     * default value <code>defaultVal</code>.
     */
    float attributeAsFloat(std::string attr,
                           float  defaultVal);

    /**
     * Quick access function to an attribute of the element or of a
     * child's child.
     */
    float getFloat(std::string path);

    /**
     * Set the comment.
     */
    void setComment(std::string comment);

#endif

    /**
     * Set the name.
     */
    void setName(std::string name);

    /**
     * Add an attribute.
     * There are no checks of any kind!
     */
    void addAttribute(std::string attributeName,
                      std::string attributeVal);

    /**
     * Add an attribute.
     * There are no checks of any kind!
     */
    void addAttribute(std::string attributeName,
                      long int    attributeVal);

    /**
     * Add a child.
     * There are no checks of any kind!
     */
    void addChild(SimpleXMLTransfer* child);


    /**
     * Sort children by a given attribute. The attributes will
     * be compared as strings.
     */
    void sortChildrenString(std::string attributeName);

    /**
     * Sort children by a given attribute. The attributes will
     * be compared as double precision floating point values.
     */
    void sortChildrenDouble(std::string attributeName);

    /**
     * Return the index of the first child with the name
     * <code>child</code> or -1 if no such child exists.
     */
    int indexOfChild(std::string child);

    /**
     * Return the index of the first child with the name
     * <code>child</code> or -1 if no such child exists.
     * Start searching at <code>nStartIdx</code>.
     */
    int indexOfChild(std::string child, int nStartIdx);

    /**
     * Adds an attribute. Throws XMLException if there already is an attribute 
     * with the same name.
     * <code>attribute</code> can be something like <code>pictures.holiday.malibu</code>
     * The last item of the path is taken as the name of the attribute. Non-existing elements
     * in the path are automatically created.
     */
    void setAttribute(std::string attribute, std::string value);

    /**
     * Adds an attribute. Throws XMLException if there already is an attribute 
     * with the same name.
     * <code>attribute</code> can be something like <code>pictures.holiday.malibu</code>
     * The last item of the path is taken as the name of the attribute. Non-existing elements
     * in the path are automatically created.
     */
    void setAttribute(std::string attribute, long int nValue);

    /**
     * Adds an attribute. If there already is an attribute
     * with the same name, it will be overwritten.
     * <code>attribute</code> can be something like <code>pictures.holiday.malibu</code>
     * The last item of the path is taken as the name of the attribute. Non-existing elements
     * in the path are automatically created.
     */
    void setAttributeOverwrite(std::string attribute, const std::string value);

    /**
     * Adds an attribute. If there already is an attribute
     * with the same name, it will be overwritten.
     * <code>attribute</code> can be something like <code>pictures.holiday.malibu</code>
     * The last item of the path is taken as the name of the attribute. Non-existing elements
     * in the path are automatically created.
     */
    void setAttributeOverwrite(std::string attribute, const long int attributeVal);

   /**
    * Makes sure that an attribute exists. If it does not, it is created with <code>defaultVal</code>.
    * <code>attribute</code> can be something like <code>pictures.holiday.malibu</code>
    * The last item of the path is taken as the name of the attribute. Non-existing elements
    * in the path are automatically created.
    */
   void makeSureAttributeExists(std::string attribute, const char* defaultVal);

   /**
    * Recursively compares <code>item</code> with itself. 
    * Returns <code>true</code> if they are equal.
    * The order of children does not matter.
    */
   bool equals(SimpleXMLTransfer* item);
   
   /**
    * Recursively compares <code>item</code> with itself. 
    * Returns <code>true</code> if they are equal.
    * The order of children does matter.
    */
   bool equalsOrdered(SimpleXMLTransfer* item);
   
    /**
     * Returns the index of the first attribute named
     * <code>attr</code> or -1 of none exists.
     */
    int indexOfAttribute(std::string attr) const;
   
   /** 
    * Returns number of attributes
    */
   int getAttributeCount() const { return(attrName.size()); };

   void overwriteAttribute(unsigned int index, std::string val);
   
   std::string attributeName(unsigned int index);
   std::string attributeVal (unsigned int index);
      
  private:
    std::string                          myName;
    std::vector<SimpleXMLTransfer*>      children;
    std::vector<std::string>             attrName;
    std::vector<std::string>             attrVal;
    std::vector<std::string>             comment;
    std::vector<int>                     commentPos;
    std::string*                         content;

    /**
     * Only for internal purposes!
     */
    void readStream(std::istream&  in,
                    int            data);

    SimpleXMLTransfer(std::istream&  in,
                      int            data);


    /**
     * Possible notation for integer-values: decimal, hexadecimal with 
     * prefixed '0x' or binary with prefixed '0b'.
     */
    int    convToInt(std::string value);

    /**
     */
    float  convToFloat(std::string value);

    double convToDouble(std::string value);

    /**
     * Converts a std::string (attribute value in an XML file)
     * into a Unicode-<code>std::string</code>.
     * <br>
     * Does not work in all cases as some things are converted
     * wrong or can cause an XMLException.
     * <br>
     * The representation of <code>in</code> is subjected to some
     * restrictions.
     * Only these entities are converted correctly:
     * <code>&lt;</code>,
     * <code>&gt;</code>,
     * <code>&amp;</code>,
     * <code>&quot;</code>
     * and characters represented as <code>&#xHEXVAL;</code>.
     */
    std::string convFromXML(std::string in);

    /**
     * Convert a std::string to make it storable as an
     * attributes' value in an XML file.
     */
    std::string convToXML(std::string in);
   
   
   /**
    * 
    */
   void print(std::ostream& out,
              int           nIndent,
              std::string   val);
   
   /**
    * describes the source of the data
    */
   std::string sourcedescr;
};

#endif
