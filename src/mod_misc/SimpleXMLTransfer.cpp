/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2003, 2004, 2005, 2006, 2008 Jens Wilhelm Wulf (original author)
 * Copyright (C) 2004, 2005, 2008 Jan Reucker
 * Copyright (C) 2005 Olivier Bordes
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
 * \file SimpleXMLTransfer.cpp
 *
 * -per ifdef ausgeklammerte Funktionen sind noch umzusetzen
 *
 * -wahrscheinlich eine schon wesentliche Änderung im Vergleich zur java-Version:
 *  im Zustand STATE_READNAME gehe ich auch bei ((c0 == '/') && myName.length() > 0)
 *  in STATE_ATTR_NAME über. Damit kann man nämlich auch so etwas wie '<br/>' behandeln,
 *  die vorige Version konnte nur '<br />'. Wie ist das aktuell in der Java-Version?
 */

#include "SimpleXMLTransfer.h"
#include "lib_conversions.h"
#include <ctype.h>
#include <iostream>
#include <cstdlib>

// 1: ?
// 2: show compare
#define DEBUG 0

#if DEBUG != 0
# include <stdio.h>
#endif

XMLException::XMLException(std::string message)
{
  myMessage = message;
}

const char *XMLException::what()
{
  return (myMessage.c_str());
}

/**
 * Element ohne Daten erstellen.
 */
SimpleXMLTransfer::SimpleXMLTransfer()
{
  myName      = "data";
  content     = (std::string *) 0;
  sourcedescr = "default constructor";
}

SimpleXMLTransfer::SimpleXMLTransfer(std::istream & in, int data)
{
  content     = (std::string *) 0;
  sourcedescr = "istream";
  readStream(in, data);
}

/**
 * Aus einer Datei lesen. <code>source</code> ist der Pfad zur Datei.
 * Wenn sie mit <code>file://</code> beginnt, wird eine lokale Datei gelesen, ansonsten
 * wird ein Netzwerkzugriff versucht.
 * Wenn der Name mit <code>.gz</code> endet, wird angenommen, dass die Datei mit GnuZip
 * gepackt ist.
 */
SimpleXMLTransfer::SimpleXMLTransfer(std::string source)
{
  std::ifstream in;

  content = (std::string*) 0;

  in.open(source.c_str());
  sourcedescr = "file: " + source;

  if (!in)
  {
    throw XMLException("Error opening " + source);
  }
  else
  {
    readStream(in, -2);
    in.close();
  }
}

SimpleXMLTransfer::SimpleXMLTransfer(std::istream& in)
{
  content = (std::string*) 0;
  sourcedescr = "stream";
  readStream(in, -2);
}

SimpleXMLTransfer::SimpleXMLTransfer(SimpleXMLTransfer* source)
{
  content     = (std::string *) 0;
  sourcedescr = "copy constructor";

  setName(source->getName());
  setContent(source->getContentString());

  for (int n=0; n<source->getAttributeCount(); n++)
    addAttribute(source->attributeName(n), source->attributeVal(n));

  for (int n=0; n<source->getChildCount(); n++)
    addChild(new SimpleXMLTransfer(source->getChildAt(n)));
}

SimpleXMLTransfer::~SimpleXMLTransfer()
{
  SimpleXMLTransfer *tmp;

  if (content != (std::string*) 0)
    delete content;

  while (children.size() > 0)
  {
    tmp = children[children.size() - 1];
    delete tmp;
    children.pop_back();
  }
}

/**
 * Nur zur internen Vewendung.
 */
void SimpleXMLTransfer::readStream(std::istream & in, int data)
{
  bool fLesen = true;
  bool fGeschlossen = false;

  std::string name;
//  std::string value;
  std::string myComment;
  char c0 = ' ';
  char c1 = ' ';
  char c2 = ' ';
  T_State nState;
  T_State nStateAfterComment = STATE_IDLE;
  std::string endTagName;

  std::string attributename;
  std::string attributeval;

  if (data != -2)
  {
    nState = STATE_READNAME;
    myName = "";
    myName.push_back((char) data);
  }
  else
    nState = STATE_IDLE;

  while (fLesen)
  {
    data = in.get();

    if (data < 0)
      fLesen = false;
    else
    {
      c0 = (char) data;
      switch (nState)
      {
          // Auf eine öffnende Klammer warten.
        case STATE_IDLE:
          if (c0 == '<')
            nState = STATE_START;
          break;

          //
        case STATE_START:
          if (c0 == '?')
            nState = STATE_INITTAG;
          else if (c0 == '!')
          {
            nStateAfterComment = STATE_IDLE;
            nState = STATE_COMMENT;
            myComment = "<!";
          }
          else
          {
            nState = STATE_READNAME;
            myName = "";
            myName.push_back(c0);
          }
          break;

          // Das Ende vom Inittag abwarten.
        case STATE_INITTAG:
          if (c0 == '>')
            nState = STATE_IDLE;
          break;

          // Das Ende vom Kommentar abwarten.
        case STATE_COMMENT:
          myComment.push_back(c0);
          if (c0 == '>' && c1 == '-' && c2 == '-')
          {
            comment.push_back(myComment);
            if (nStateAfterComment == STATE_IDLE)
              commentPos.push_back(-1);
            else
              commentPos.push_back(children.size());
            nState = nStateAfterComment;
          }
          break;

          // Den Namen einlesen
        case STATE_READNAME:
          if (c0 == '=')
          {
            throw
            XMLException
            ("Found \'=\' without preceding attributename in " +
             myName);
          }
          else
            if ((c0 == '/' || c0 == ' ' || c0 == '\n' || c0 == '\r'
                 || c0 == '\t') && myName.length() > 0)
            {
              nState = STATE_ATTR_NAME;
              attributename = "";
            }
            else
            {
              if (c0 == '>')
              {
                nState = STATE_WAIT_CONT;
              }
              else
                myName.push_back(c0);
            }
          break;

        case STATE_ATTR_NAME:
          if (c0 == '=')
          {
            if (attributename.length() == 0)
            {
              // Es wurde versucht, einen Wert zuzuweisen, obwohl noch kein Attribut
              // definiert wurde.
              // Vielleicht wurde versucht, dem Element einen Wert zuzuweisen?
              throw
              XMLException
              ("Found \'=\' without preceding attributename in " +
               myName);
            }
            nState = STATE_ATTR_VAL_START;
          }
          else
          {
            if (c0 == '>')
            {
              if (c1 == '/')
              {
                fGeschlossen = true;
                fLesen = false;
              }
              else
                nState = STATE_WAIT_CONT;
            }
            else
            {
              if (isspace(c0) == false)
                attributename.push_back(c0);
              else
              {
                if (attributename.length() > 0)
                {
                  // Ich hatte bereits einen Namen eingelesen, jetzt kommt Whitespace.
                  // Also ist der Name beendet, es muss jetzt das Gleichheitszeichen
                  // und der Wert kommen.
                  nState = STATE_ATTR_VAL_WAIT_EQ;
                }
              }
            }
          }
          break;

        case STATE_ATTR_VAL_WAIT_EQ:
          if (c0 == '=')
            nState = STATE_ATTR_VAL_START;
          else if (isspace(c0) == false)
          {
            throw XMLException("Element " + myName +
                               ": expected \'=\', got \'" + c0 +
                               "\' after attribute " + attributename);
          }
          break;

        case STATE_ATTR_VAL_START:
          if (c0 == '"')
          {
            attributeval = "";
            nState = STATE_ATTR_VAL;
          }
          break;

        case STATE_ATTR_VAL:
          if (c0 == '"')
          {
            if (attributename.length() > 0)
              addAttribute(attributename, convFromXML(attributeval));
            attributeval = "";

            nState = STATE_ATTR_NAME;
            attributename = "";
          }
          else
            attributeval.push_back(c0);
          break;

        case STATE_WAIT_CONT:
          if (c0 == '<')
            nState = STATE_CONT_START;
          else
          {
            if (content == (std::string *) 0 && isspace(c0) == false)
              content = new std::string();
            if (content != (std::string *) 0)
              content->push_back(c0);
          }
          break;

        case STATE_CONT_START:
          if (c0 == '/')
            nState = STATE_ENDTAG;
          else if (c0 == '!')
          {
            nStateAfterComment = STATE_WAIT_CONT;
            nState = STATE_COMMENT;
            myComment = "<!";
          }
          else
          {
            SimpleXMLTransfer *child =
              new SimpleXMLTransfer(in, (int) c0);
            addChild(child);
            nState = STATE_WAIT_CONT;
          }
          break;

          // Ich warte auf das Ende und überprüfe, ob auch
          // mein Name wieder korrekt im End-Tag steht.
        case STATE_ENDTAG:
          if (c0 == '>')
          {
            if (endTagName.compare(getName()) == 0)
            {
              fGeschlossen = true;
              fLesen = false;
            }
            else
            {
              throw XMLException("XML-Element " + getName()
                                 +
                                 " wurde nicht korrekt abgeschlossen: "
                                 + endTagName);
            }
          }
          else
            endTagName.push_back(c0);
          break;

        default:
          break;
      }

      c2 = c1;
      c1 = c0;
    }
  }

#if DEBUG == 1
  if (content != (std::string *) 0)
    printf("Content length=%i: %s", content->length(), content->c_str());
  else
    printf("No content\n");
#endif

  // trailing whitespace vom content entfernen
  if (content != (std::string *) 0)
    while (content->length() > 0
           && isspace(content->at(content->length() - 1)) == true)
      content->replace(content->length() - 1, 1, "");

  if (fGeschlossen == false)
  {
    throw XMLException(sourcedescr + ": XML-Element wurde nicht abgeschlossen: " +
                       getName());
  }
}

/**
 * Liefert den Namen des Elements zurück.
 */
std::string SimpleXMLTransfer::getName()
{
  return (myName);
}

/**
 * Liefert den Inhalt als String zurück
 */
std::string SimpleXMLTransfer::getContentString()
{
  if (content != (std::string *) 0)
    return (*content);
  else
    return ("");
}

/**
 * Liefert den Wert des Attributs <code>attr</code> zurück oder
 * <code>defaultVal</code>, wenn es nicht existiert.
 */
double
SimpleXMLTransfer::attributeAsDouble(std::string attr, double defaultVal)
{
  int index = indexOfAttribute(attr);

  if (index < 0)
  {
    return (defaultVal);
  }
  else
  {
    return (convToDouble(attrVal[index]));
  }
}

/**
 * Liefert den Wert des Attributs <code>attr</code> zurück oder
 * eine XMLException, wenn es nicht existiert.
 */
double SimpleXMLTransfer::attributeAsDouble(unsigned int index)
{
  if (index > attrName.size())
  {
    throw XMLException("No such attribute in " + getName());
  }
  else
  {
    return (convToDouble(attrVal[index]));
  }
}

/**
 * Liefert den Wert des Attributs <code>attr</code> zurück oder
 * eine XMLException, wenn es nicht existiert.
 */
double SimpleXMLTransfer::attributeAsDouble(std::string attr)
{
  int index = indexOfAttribute(attr);

  if (index < 0)
  {
    throw XMLException("No Attribute named " + attr + " in " + getName());
  }
  else
  {
    return (convToDouble(attrVal[index]));
  }
}

/**
 * Liefert den Inhalt als Double zurück
 */
double SimpleXMLTransfer::getContentDouble()
{
  return (convToDouble(*content));
}

/**
 * Liefert den Inhalt als Int zurück
 */
int SimpleXMLTransfer::getContentInt()
{
  return (convToInt(*content));
}

/**
 * Schnellfunktion zum Zugriff auf ein Attribut dieses Elements oder von
 * einem Kindeskind.
 */
std::string SimpleXMLTransfer::getString(std::string path)
{
  std::string::size_type pos = path.rfind('.');

  if (pos != std::string::npos)
  {
    SimpleXMLTransfer *item = getChild(path.substr(0, pos));
    return (item->attribute(path.substr(pos + 1)));
  }
  else
  {
    return (attribute(path));
  }
}

/**
 * Schnellfunktion zum Zugriff auf ein Attribut dieses Elements oder von
 * einem Kindeskind.
 *
 * Mögliche Notation für einen Integer-Wert: dezimal, hexadezimal mit
 * vorangestelltem '0x' oder binär mit vorangestelltem '0b'.
 */
int SimpleXMLTransfer::getInt(std::string path)
{
  std::string::size_type pos = path.rfind('.');

  if (pos != std::string::npos)
  {
    SimpleXMLTransfer *item = getChild(path.substr(0, pos));
    return (item->attributeAsInt(path.substr(pos + 1)));
  }
  else
  {
    return (attributeAsInt(path));
  }
}

/**
 * Schnellfunktion zum Zugriff auf ein Attribut dieses Elements oder von
 * einem Kindeskind.
 */
double SimpleXMLTransfer::getDouble(std::string path)
{
  std::string::size_type pos = path.rfind('.');

  if (pos != std::string::npos)
  {
    SimpleXMLTransfer *item = getChild(path.substr(0, pos));
    return (item->attributeAsDouble(path.substr(pos + 1)));
  }
  else
  {
    return (attributeAsDouble(path));
  }
}

std::string SimpleXMLTransfer::getString(std::string path, std::string stringDefault)
{
  try
  {
    std::string::size_type pos = path.rfind('.');

    if (pos != std::string::npos)
    {
      SimpleXMLTransfer *item = getChild(path.substr(0, pos));
      return (item->attribute(path.substr(pos + 1)));
    }
    else
    {
      return (attribute(path));
    }
  }
  catch (XMLException e)
  {
    makeSureAttributeExists(path, stringDefault.c_str());
    return(stringDefault);
  }
}

/**
 * Schnellfunktion zum Zugriff auf ein Attribut dieses Elements oder von
 * einem Kindeskind.
 *
 * Mögliche Notation für einen Integer-Wert: dezimal, hexadezimal mit
 * vorangestelltem '0x' oder binär mit vorangestelltem '0b'.
 */
int SimpleXMLTransfer::getInt(std::string path, int nDefault)
{
  try
  {
    std::string::size_type pos = path.rfind('.');

    if (pos != std::string::npos)
    {
      SimpleXMLTransfer *item = getChild(path.substr(0, pos));
      return (item->attributeAsInt(path.substr(pos + 1)));
    }
    else
    {
      return (attributeAsInt(path));
    }
  }
  catch (XMLException e)
  {
    makeSureAttributeExists(path, itoStr(nDefault, ' ', 1).c_str());
    return(nDefault);
  }
}

/**
 * Schnellfunktion zum Zugriff auf ein Attribut dieses Elements oder von
 * einem Kindeskind.
 */
double SimpleXMLTransfer::getDouble(std::string path, double dDefault)
{
  try
  {
    std::string::size_type pos = path.rfind('.');

    if (pos != std::string::npos)
    {
      SimpleXMLTransfer *item = getChild(path.substr(0, pos));
      return (item->attributeAsDouble(path.substr(pos + 1)));
    }
    else
    {
      return (attributeAsDouble(path));
    }
  }
  catch (XMLException e)
  {
    makeSureAttributeExists(path, doubleToString(dDefault).c_str());
    return(dDefault);
  }
}

#if 1 == 2

/**
 * Liefert den Wert des Attributs <code>attr</code> zurück oder
 * <code>defaultVal</code>, wenn es nicht existiert.
 */
float SimpleXMLTransfer::attributeAsFloat(std::string attr, float defaultVal)
{
  int index = indexOfAttribute(attr);

  if (index < 0)
    return (defaultVal);
  else
    return (convToFloat((String) attrVal.elementAt(index)));
}

/**
 * Schnellfunktion zum Zugriff auf ein Attribut dieses Elements oder von
 * einem Kindeskind.
 */
float SimpleXMLTransfer::getFloat(std::string path)
{
  int pos = path.lastIndexOf('.');

  if (pos > 0)
  {
    SimpleXMLTransfer item = getChild(path.substring(0, pos));
    return (item.attributeAsFloat(path.substring(pos + 1)));
  }
  else
    return (attributeAsFloat(path));
}

/**
 * Setzt den Kommentar.
 */
void SimpleXMLTransfer::setComment(std::string comment)
{
  myComment = "<!-- ";
  myComment.append(comment);
  myComment.append(" -->");
}


#endif


void SimpleXMLTransfer::setAttribute(std::string attribute, std::string value)
{
  if (attribute.find('.') != std::string::npos)
  {
    int index;
    std::string childName;
    SimpleXMLTransfer* item = this;
    std::string::size_type pos;

    while ((pos = attribute.find('.')) != std::string::npos)
    {
      childName = attribute.substr(0, pos);
      attribute = attribute.substr(pos + 1);

      index = item->indexOfChild(childName);
      if (index < 0)
      {
        SimpleXMLTransfer* tmp = new SimpleXMLTransfer();
        tmp->setName(childName);
        item->addChild(tmp);
        item = tmp;
      }
      else
        item = item->getChildAt(index);
    }
    item->setAttribute(attribute, value);
  }
  else
  {
    if (indexOfAttribute(attribute) >= 0)
      throw XMLException("Attribute named " + attribute +
                         " already exists in " + getName());

    addAttribute(attribute, value);
  }
}

void SimpleXMLTransfer::setAttribute(std::string attribute, long int nValue)
{
  setAttribute(attribute, itoStr(nValue, ' ', 1));
}

void SimpleXMLTransfer::setAttributeOverwrite(std::string attribute, std::string value)
{
  if (attribute.find('.') != std::string::npos)
  {
    int index;
    std::string childName;
    SimpleXMLTransfer* item = this;
    std::string::size_type pos;

    while ((pos = attribute.find('.')) != std::string::npos)
    {
      childName = attribute.substr(0, pos);
      attribute = attribute.substr(pos + 1);

      index = item->indexOfChild(childName);
      if (index < 0)
      {
        SimpleXMLTransfer* tmp = new SimpleXMLTransfer();
        tmp->setName(childName);
        item->addChild(tmp);
        item = tmp;
      }
      else
        item = item->getChildAt(index);
    }

    item->setAttributeOverwrite(attribute, value);
  }
  else
  {
    int index = indexOfAttribute(attribute);
    if (index >= 0)
    {
      attrName.erase(attrName.begin() + index);
      attrVal.erase(attrVal.begin() + index);
    }

    addAttribute(attribute, value);
  }
}

void SimpleXMLTransfer::setAttributeOverwrite(std::string attribute, long int nValue)
{
  setAttributeOverwrite(attribute, itoStr(nValue, ' ', 1));
}

void SimpleXMLTransfer::makeSureAttributeExists(std::string attribute, const char* defaultVal)
{
  if (attribute.find('.') != std::string::npos)
  {
    int index;
    std::string childName;
    SimpleXMLTransfer* item = this;
    std::string::size_type pos;

    while ((pos = attribute.find('.')) != std::string::npos)
    {
      childName = attribute.substr(0, pos);
      attribute = attribute.substr(pos + 1);

      index = item->indexOfChild(childName);
      if (index < 0)
      {
        SimpleXMLTransfer* tmp = new SimpleXMLTransfer();
        tmp->setName(childName);
        item->addChild(tmp);
        item = tmp;
      }
      else
        item = item->getChildAt(index);
    }

    item->makeSureAttributeExists(attribute, defaultVal);
  }
  else
  {
    int index = indexOfAttribute(attribute);
    if (index < 0)
    {
      addAttribute(attribute, defaultVal);
    }
  }
}

/**
 * Konvertiert einen String (Wert eines Attributes in einer XML-Datei)
 * in einen Unicode-<code>String</code>.
 * <br>
 * Funktioniert nicht allgemeingültig, da bestimmte
 * Dinge falsch umgewandelt werden oder zu einer XMLException führen.
 * <br>
 * Die Darstellung in <code>in</code> unterliegt also einigen
 * Einschränkungen.
 * Korrekt umgewandelt werden nur:
 * <code>&lt;</code>,
 * <code>&gt;</code>,
 * <code>&amp;</code>,
 * <code>&quot;</code>
 * sowie Zeichen in der Darstellung<code>&#xHEXWERT;</code>.
 */
std::string SimpleXMLTransfer::convFromXML(std::string in)
{
  int length = in.length();
  std::string out;
  char ch;

  for (int i = 0; i < length;)
  {
    ch = in.at(i);

    if (ch != '&')
    {
      out.push_back(ch);
      i++;
    }
    else
    {
      std::string sc = "";

      i++;
      while ((ch = in.at(i++)) != ';')
        sc.push_back(ch);

      if (sc.length() < 2)
      {
        throw XMLException("Error converting character: " + sc);
      }
      else
      {
        ch = sc.at(0);

        if (ch == 'l') // lt
          out.push_back('<');
        else if (ch == 'g') // gt
          out.push_back('>');
        else if (ch == 'a') // amp
          out.push_back('&');
        else if (ch == 'q') // quot
          out.push_back('\"');
        else if (ch != '#')
        {
          throw XMLException("Error converting character: " + sc);
        }
        else
        {
          out.push_back((char) hex2int(sc.substr(2)));
        }
      }
    }
  }

  return (out);
}

/**
 * Konvertiert einen String, so dass er in einer XML-Datei
 * als Wert eines Attributes gespeichert werden kann.
 * Die Konvertierung ist nicht entsprechend Unicode, also
 * letztlich doch abhängig vom Zeichensatz des Systems.
 */
std::string SimpleXMLTransfer::convToXML(std::string in)
{
  int length = in.length();
  std::string out;
  char ch;

  for (int i = 0; i < length; i++)
  {
    ch = in.at(i);

    if ((ch <= '~' && ch >= '?') ||
        (ch <= ';' && ch >= '(') ||
        (ch <= '%' && ch >= '#') ||
        (ch == ' ') ||
        (ch == '!') ||
        (ch == '='))
    {
      out.push_back(ch);
    }
    else
    {
      out.append("&#x");

      // Das führte zu ungültigen Codes (signed int, negativ):
      // out += itoHexStr(  (int)(ch)    );

      // So ist es korrekt:
      out += itoHexStr(  (int)((unsigned char)(ch))    );

      out.push_back(';');
    }
  }

  return (out);
}

/**
 * Fügt ein Attribut hinzu.
 * Es gibt keinerlei Überprüfung.
 */
void
SimpleXMLTransfer::addAttribute(std::string attributeName,
                                std::string attributeVal)
{
  //       System.out.println("SimpleXMLTransfer.addAttribute(" + attributeName + ", " + attributeVal + ")");
  //
  attrName.push_back(attributeName);
  attrVal.push_back(attributeVal);
}

/**
 * Fügt ein Attribut hinzu.
 * Es gibt keinerlei Überprüfung.
 */
void
SimpleXMLTransfer::addAttribute(std::string attributeName,
                                long int attributeVal)
{
  addAttribute(attributeName, itoStr(attributeVal, ' ', 1));
}

/**
 * Fügt ein Kind hinzu.
 * Es gibt keinerlei Überprüfung.
 */
void SimpleXMLTransfer::addChild(SimpleXMLTransfer * child)
{
  children.push_back(child);
}

void SimpleXMLTransfer::print()
{
  std::cout << "<?xml version=\"1.0\" ?>\n";

  print(std::cout, 0);
}

void SimpleXMLTransfer::print(std::ostream & out)
{
  out << "<?xml version=\"1.0\" ?>\n";

  print(out, 0);
}

void SimpleXMLTransfer::print(std::ostream& out,
                              int           nIndent,
                              std::string   val)
{
  for (int n = 0; n < nIndent; n++)
    out << "  ";

  out << val;
}

void SimpleXMLTransfer::print(std::ostream & out, int nIndent)
{
  int nCMax = -1;

  if (comment.size() > 0 && commentPos[0] == -1)
  {
    print(out, nIndent, comment[0] + "\n");
    nCMax = 0;
  }

  // Es könnte sein, dass dieses Element ein reiner Kommentar ist!
  if (getName().length() > 0)
  {
    // mich selbst eröffnen
    print(out, nIndent, "<" + getName());

    // Attribute schreiben
    if (attrName.size() > 0)
    {
      int nSpalte = nIndent * 2 + getName().length();
      std::string aVal;
      std::string aName;

      for (unsigned int i = 0; i < attrName.size(); i++)
      {
        aVal = convToXML(attrVal[i]);
        aName = attrName[i];

        if (nSpalte + aVal.length() + aName.length() > 80)
        {
          out << "\n";

          for (int n = 0; n < nIndent; n++)
            out << "  ";

          out << "  ";
          nSpalte = nIndent * 2 + 2;
        }
        else
          nSpalte += aVal.length() + aName.length() + 4;

        out << " " << aName << "=\"" << aVal << "\"";
      }
    }

    if (content != (std::string *) 0 || children.size() > 0)
    {
      out << ">";

      // Wenn ich irgendwelchen Inhalt habe, schreibe ich den vor den Kindern
      // und den Kommentaren.
      if (content != (std::string *) 0)
      {
        out << getContentString();
      }

      if (comment.size() > (unsigned int)(nCMax+1))
      {
        if (commentPos[nCMax+1] == 0)
        {
          nCMax++;
          out << "\n";
          print(out, nIndent, comment[nCMax]);
        }
      }

      // Die Kinder sollen ihre Daten schreiben, falls es welche gibt.
      if (children.size() > 0)
      {
        out << "\n";

        for (unsigned int i = 0; i < children.size(); i++)
        {
          children[i]->print(out, nIndent + 1);
          for (unsigned int m=0; m<comment.size(); m++)
          {
            if (commentPos[m] == (int)i+1)
              print(out, nIndent, comment[m] + "\n");
          }
        }

        // ich bin fertig
        for (int n = 0; n < nIndent; n++)
          out << "  ";
      }
      out << "</" << getName() << ">\n";
    }
    else
      // es gibt keine Kinder
      out << " />\n";
  }
}

/**
 * Liefert das Kind mit dem Index <code>childIndex</code> zurück.
 */
SimpleXMLTransfer *SimpleXMLTransfer::getChildAt(int childIndex)
{
  if (children.size() == 0)
    return ((SimpleXMLTransfer *) 0);
  return (children[childIndex]);
}

/**
 * Liefert die Anzahl der Kinder zurück.
 */
int SimpleXMLTransfer::getChildCount()
{
  return (children.size());
}

/**
 * Liefert den Wert des Attributs <code>attr</code> zurück oder
 * eine XMLException, wenn es nicht existiert.
 */
int SimpleXMLTransfer::attributeAsInt(std::string attr)
{
  int index = indexOfAttribute(attr);

  if (index < 0)
  {
    throw XMLException("No Attribute named " + attr + " in " + getName());
  }
  else
  {
    return (convToInt(attrVal[index]));
  }
}

/**
 * Liefert den Wert des Attributs <code>attr</code> zurück oder
 * <code>defaultVal</code>, wenn es nicht existiert.
 */
int SimpleXMLTransfer::attributeAsInt(std::string attr, int defaultVal)
{
  int index = indexOfAttribute(attr);

  if (index < 0)
  {
    return (defaultVal);
  }
  else
  {
    return (convToInt(attrVal[index]));
  }
}

/**
 * Liefert den Wert des Attributs <code>attr</code> zurück oder
 * eine XMLException, wenn es nicht existiert.
 */
float SimpleXMLTransfer::attributeAsFloat(std::string attr)
{
  int index = indexOfAttribute(attr);

  if (index < 0)
  {
    throw XMLException("No Attribute named " + attr + " in " + getName());
  }
  else
  {
    return (convToFloat(attrVal[index]));
  }
}

/**
 * Liefert den Wert des Attributs <code>attr</code> zurück oder
 * <code>defaultVal</code>, wenn es nicht existiert.
 */
std::string SimpleXMLTransfer::attribute(std::string attr,
    std::string defaultVal)
{
  int index = indexOfAttribute(attr);

  if (index < 0)
    return (defaultVal);
  else
    return (attrVal[index]);
}

/**
 * Liefert den Wert des Attributs <code>attr</code> zurück oder
 * eine XMLException, wenn es nicht existiert.
 */
std::string SimpleXMLTransfer::attribute(std::string attr)
{
  int index = indexOfAttribute(attr);

  if (index < 0)
  {
    throw XMLException("No Attribute named " + attr + " in " + getName());
  }
  else
  {
    return (attrVal[index]);
  }
}

/**
 * Liefert den Index des ersten Attributes mit dem Namen
 * <code>attr</code> zurück oder -1, wenn es keins gibt.
 */
int SimpleXMLTransfer::indexOfAttribute(std::string attr) const
{
#if DEBUG == 1
  printf("int SimpleXMLTransfer::indexOfAttribute(\"%s\")\n", attr.c_str());
#endif
  if (attrName.size() == 0)
    return (-1);

  int index = -1;

  for (unsigned int i = 0; i < attrName.size() && index == -1; i++)
    if (attr.compare(attrName[i]) == 0)
      index = i;

#if DEBUG == 1
  printf("int SimpleXMLTransfer::indexOfAttribute(\"%s\") = %i\n",
         attr.c_str(), index);
#endif
  return (index);
}

double SimpleXMLTransfer::convToDouble(std::string value)
{
  char*       ptr;
  double      tmp;
  const char* inptr = value.c_str();

  tmp = strtod(inptr, &ptr);
  if (ptr == inptr || *ptr != '\0')
  {
    value = "convToDouble: no number or trailing garbage in " + value;
    throw XMLException(value.c_str());
  }
  return(tmp);
}

float SimpleXMLTransfer::convToFloat(std::string value)
{
  return(convToDouble(value));
}

/**
 * Mögliche Notation für einen Integer-Wert: dezimal, hexadezimal mit
 * vorangestelltem '0x' oder binär mit vorangestelltem '0b'.
 */
int SimpleXMLTransfer::convToInt(std::string value)
{
  long        nVal;
  char*       ptr;
  const char* inptr;
  int         base;

  if (value.find("0b") == 0 && value.length() > 2)
  {
    // binäre Darstellung
    inptr = value.c_str()+2;
    base  = 2;
  }
  else if (value.length() > 1 && toupper(value[1]) == 'X')
  {
    // hexadezimale Darstellung
    inptr = value.c_str()+2;
    base  = 16;
  }
  else
  {
    // dezimale Darstellung
    inptr = value.c_str();
    base  = 10;
  }

  nVal = strtol(inptr, &ptr, base);
  if (ptr == inptr || *ptr != '\0')
  {
    value = "convToInt: no number or trailing garbage in " + value;
    throw XMLException(value.c_str());
  }
  return(nVal);
}

/**
 * Liefert das erste Kind mit dem Namen <code>child</code> zurück
 * oder eine <code>XMLException</code>, wenn es keins gab.
 *
 * Zugriff auf Kindeskinder ist erlaubt, also
 * <code>getChild("essen.morgens.brot")</code>
 * anstatt
 * <code>getChild("essen").getChild("morgens").getChild("brot")</code>
 */
SimpleXMLTransfer *SimpleXMLTransfer::getChild(std::string child, bool fCreate)
{
  int                    index;
  SimpleXMLTransfer*     tmp;
  std::string::size_type pos;

  pos = child.find('.');

  if (pos != std::string::npos)
  {
    std::string parentName = "";
    std::string childName = "";
    SimpleXMLTransfer* item = this;

    do
    {
      childName = child.substr(0, pos);
      child     = child.substr(pos + 1);
      pos       = child.find('.');

      index = item->indexOfChild(childName);
      if (index < 0)
      {
        if (fCreate)
        {
          tmp = new SimpleXMLTransfer();
          tmp->setName(childName);
          item->addChild(tmp);
          index = item->indexOfChild(childName);
        }
        else
          throw XMLException("No item named " + childName + " in " +
                             parentName);
      }

      if (parentName.length() > 0)
        parentName.push_back('.');
      parentName.append(childName);

      item = item->getChildAt(index);
    }
    while (pos != std::string::npos);

    index = item->indexOfChild(child);
    if (index < 0)
    {
      if (fCreate)
      {
        tmp = new SimpleXMLTransfer();
        tmp->setName(child);
        item->addChild(tmp);
        return(tmp);
      }
      else
        throw XMLException("No item named " + child + " in " + parentName);
    }

    return (item->getChildAt(index));
  }
  else
  {
    index = indexOfChild(child);

    if (index < 0)
    {
      if (fCreate)
      {
        tmp = new SimpleXMLTransfer();
        tmp->setName(child);
        addChild(tmp);
        return(tmp);
      }
      else
        throw XMLException("No child named " + child + " in " + getName());
    }
    else
    {
      return (children[index]);
    }
  }
}

/**
 * Liefert den Index des ersten Kindes mit dem Namen
 * <code>child</code> zurück oder -1, wenn es keins gibt.
 */
int SimpleXMLTransfer::indexOfChild(std::string child)
{
  if (children.size() == 0)
    return (-1);

  int index = -1;
  int size  = children.size();

  for (int i = 0; i < size && index == -1; i++)
    if (child.compare(children[i]->getName()) == 0)
      index = i;

  return (index);
}

int SimpleXMLTransfer::indexOfChild(std::string child, int nStartIdx)
{
  if (children.size() == 0)
    return (-1);

  if (nStartIdx < 0 || nStartIdx >= (int)(children.size()))
    nStartIdx = 0;

  int index = -1;
  int size  = children.size();

  for (int i = nStartIdx; i < size && index == -1; i++)
    if (child.compare(children[i]->getName()) == 0)
      index = i;

  if (index < 0)
  {
    for (int i = 0; i < nStartIdx && index == -1; i++)
      if (child.compare(children[i]->getName()) == 0)
        index = i;
  }

  return (index);
}

void SimpleXMLTransfer::removeChildAt(int nIndex)
{
  if (children.size() <= (unsigned int) nIndex)
    throw
    XMLException
    ("void SimpleXMLTransfer::removeChildAt(int nIndex): Index out of bounds in "
     + getName());

  children.erase(children.begin() + nIndex);
}

void SimpleXMLTransfer::removeChild(SimpleXMLTransfer* XMLPtr)
{
  bool fErr   = true;

  if (children.size())
  {
    for (unsigned int nIndex=0; nIndex<children.size(); nIndex++)
    {
      if (children[nIndex] == XMLPtr)
      {
        fErr = false;
        children.erase(children.begin() + nIndex);
        nIndex = children.size() + 1;
      }
    }
  }

  if (fErr)
    throw
    XMLException
    ("void SimpleXMLTransfer::removeChild(...): no such child");
}

void SimpleXMLTransfer::replaceChild(SimpleXMLTransfer* XMLPtrOld,
                                     SimpleXMLTransfer* XMLPtrNew)
{
  bool fErr   = true;

  if (children.size())
  {
    for (unsigned int nIndex=0; nIndex<children.size(); nIndex++)
    {
      if (children[nIndex] == XMLPtrOld)
      {
        fErr = false;
        children[nIndex] = XMLPtrNew;
        nIndex = children.size() + 1;
      }
    }
  }

  if (fErr)
    throw
    XMLException
    ("void SimpleXMLTransfer::replaceChild(...): no such child");
}

/**
 * Setzt den Namen.
 */
void SimpleXMLTransfer::setName(std::string name)
{
  myName = name;
}

/**
 * Setzt den Inhalt.
 */
void SimpleXMLTransfer::setContent(std::string newContent)
{
  if (content != (std::string *) 0)
    delete content;

  content = new std::string(newContent);
}

void SimpleXMLTransfer::sortChildrenString(std::string attributeName)
{
  bool fChanged = true;
  int n;
  int size;
  std::string attr1;
  std::string attr2;
  SimpleXMLTransfer *tmp;

  size = children.size();

  while (fChanged)
  {
    fChanged = false;

    for (n = 0; n < size - 1; n++)
    {
      attr1 = children[n]->attribute(attributeName, "");
      attr2 = children[n + 1]->attribute(attributeName, "");

      if (attr1.compare(attr2) > 0)
      {
        tmp = children[n];
        children[n] = children[n + 1];
        children[n + 1] = tmp;
        fChanged = true;
      }
    }

    if (fChanged == true)
    {
      fChanged = false;

      for (n = size - 2; n >= 0; n--)
      {
        attr1 = children[n]->attribute(attributeName, "");
        attr2 = children[n + 1]->attribute(attributeName, "");

        if (attr1.compare(attr2) > 0)
        {
          tmp = children[n];
          children[n] = children[n + 1];
          children[n + 1] = tmp;
          fChanged = true;
        }
      }
    }
  }
}

void SimpleXMLTransfer::sortChildrenDouble(std::string attributeName)
{
  bool fChanged = true;
  int n;
  int size;
  double attr1;
  double attr2;
  SimpleXMLTransfer *tmp;

  size = children.size();

  while (fChanged)
  {
    fChanged = false;

    for (n = 0; n < size - 1; n++)
    {
      attr1 = children[n]->attributeAsDouble(attributeName, 0);
      attr2 = children[n + 1]->attributeAsDouble(attributeName, 0);

      if (attr1 > attr2)
      {
        tmp = children[n];
        children[n] = children[n + 1];
        children[n + 1] = tmp;
        fChanged = true;
      }
    }

    if (fChanged == true)
    {
      fChanged = false;

      for (n = size - 2; n >= 0; n--)
      {
        attr1 = children[n]->attributeAsDouble(attributeName, 0);
        attr2 = children[n + 1]->attributeAsDouble(attributeName, 0);

        if (attr1 > attr2)
        {
          tmp = children[n];
          children[n] = children[n + 1];
          children[n + 1] = tmp;
          fChanged = true;
        }
      }
    }
  }
}

void SimpleXMLTransfer::delAttribute(std::string attribute)
{
  int n = indexOfAttribute(attribute);
  if (n < 0)
    throw XMLException("Attribute named " + attribute +
                       " does not exist in " + getName());

  attrName.erase(attrName.begin() + n);
  attrVal.erase(attrVal.begin() + n);
}

bool SimpleXMLTransfer::equals(SimpleXMLTransfer* item)
{
  int                nSize;
  int                nSizeB;
  SimpleXMLTransfer* itA;
  SimpleXMLTransfer* itB;
  bool               fFound;

  // --- Compare name -------------------------------------
  if (getName().compare(item->getName()))
  {
#if DEBUG == 2
    printf("Name differs: %s != %s\n", getName().c_str(), item->getName().c_str());
#endif
    return(false);
  }

  // --- Compare attributes -------------------------------
  // Number of attributes
  nSize = getAttributeCount();
  if (nSize != item->getAttributeCount())
  {
#if DEBUG == 2
    printf("different number of attributes\n");
#endif
    return(false);
  }
  // If we have the same number of attributes, it is OK
  // if my attributes can be found and are equal to those
  // of item.
  for (int n=0; n<nSize; n++)
  {
    if (item->indexOfAttribute(attrName[n]) == -1)
    {
#if DEBUG == 2
      printf("No attribute %s\n", attrName[n].c_str());
#endif
      return(false);
    }

    if (item->attribute(attrName[n]).compare(attrVal[n]))
    {
#if DEBUG == 2
      printf("%s: %s != %s\n", attrName[n].c_str(), attrVal[n].c_str(), item->attribute(attrName[n]).c_str());
#endif
      return(false);
    }
  }

  // --- Compare children ---------------------------------
  // Number of children
  nSize = getChildCount();
  if (nSize != item->getChildCount())
  {
#if DEBUG == 2
    printf("different number of children\n");
#endif
    return(false);
  }

  // name and content
  nSizeB = item->getChildCount();
  for (int n=0; n<nSize; n++)
  {
    itA = children[n];
    // As order does not matter, I have to compare to every child
    // of 'item'.
    fFound = false;
    for (int m=0; m<nSizeB && !fFound; m++)
    {
      itB = item->getChildAt(m);
      if (itA->equals(itB))
        fFound = true;
    }
    if (!fFound)
      return(false);
  }

  return(true);
}


bool SimpleXMLTransfer::equalsOrdered(SimpleXMLTransfer* item)
{
  int                nSize;
  SimpleXMLTransfer* itA;
  SimpleXMLTransfer* itB;

  // --- Compare name -------------------------------------
  if (getName().compare(item->getName()))
  {
#if DEBUG == 2
    printf("Name differs: %s != %s\n", getName().c_str(), item->getName().c_str());
#endif
    return(false);
  }

  // --- Compare attributes -------------------------------
  // Number of attributes
  nSize = getAttributeCount();
  if (nSize != item->getAttributeCount())
  {
#if DEBUG == 2
    printf("different number of attributes\n");
#endif
    return(false);
  }
  // If we have the same number of attributes, it is OK
  // if my attributes can be found and are equal to those
  // of item.
  for (int n=0; n<nSize; n++)
  {
    if (item->indexOfAttribute(attrName[n]) == -1)
    {
#if DEBUG == 2
      printf("No attribute %s\n", attrName[n].c_str());
#endif
      return(false);
    }

    if (item->attribute(attrName[n]).compare(attrVal[n]))
    {
#if DEBUG == 2
      printf("%s: %s != %s\n", attrName[n].c_str(), attrVal[n].c_str(), item->attribute(attrName[n]).c_str());
#endif
      return(false);
    }
  }

  // --- Compare children ---------------------------------
  // Number of children
  nSize = getChildCount();
  if (nSize != item->getChildCount())
  {
#if DEBUG == 2
    printf("different number of children\n");
#endif
    return(false);
  }

  // name and content
  for (int n=0; n<nSize; n++)
  {
    itA = children[n];
    itB = item->getChildAt(n);
    if (itA->equalsOrdered(itB) == false)
      return(false);
  }

  return(true);
}

void SimpleXMLTransfer::overwriteAttribute(unsigned int index, std::string val)
{
  if (index > attrName.size())
    throw XMLException("No such attribute in " + getName());
  else
    attrVal[index] = val;
}

std::string SimpleXMLTransfer::attributeName(unsigned int index)
{
  if (index > attrName.size())
    throw XMLException("No such attribute in " + getName());
  else
    return(attrName[index]);
}

std::string SimpleXMLTransfer::attributeVal(unsigned int index)
{
  if (index > attrVal.size())
    throw XMLException("No such attribute in " + getName());
  else
    return(attrVal[index]);
}

