/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2006, 2007 Jan Reucker (original author)
 * Copyright (C) 2008 Jens Wilhelm Wulf
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
  

// puaScrListBox.cpp - implementation of a scrolling list box.
// Filename in FG sources: puList.cxx

#include "puaScrListBox.h"

/**
 * Create a puaCRRCListBox
 *
 * The constructor only prepares the color values for the highlighted entry.
 * By adjusting "high" and "label", these two color values can be mixed.
 * Right now it's set to 100% highlight color, 0% label color.
 */
puaCRRCListBox::puaCRRCListBox (int x, int y, int w, int h, char ** contents)
: puListBox(x, y, w, h, contents)
{
/*TODO
The visibility of the selected element is even less good with textured fonts.
It would be necessary to invert the color of the background of this element.
Temporarily, I force the color in red. ( J.L).*/
#if 0
  float high = 1.0f;
  float label = 0.0f;
  float frac = high + label;
  
  selectioncol[0] = (high * colour [ PUCOL_HIGHLIGHT ][0] + label * colour [ PUCOL_LABEL ][0]) / frac;
  selectioncol[1] = (high * colour [ PUCOL_HIGHLIGHT ][1] + label * colour [ PUCOL_LABEL ][1]) / frac;
  selectioncol[2] = (high * colour [ PUCOL_HIGHLIGHT ][2] + label * colour [ PUCOL_LABEL ][2]) / frac;
  selectioncol[3] = (high * colour [ PUCOL_HIGHLIGHT ][3] + label * colour [ PUCOL_LABEL ][3]) / frac;
#else
//HIGHLIGHT red !!
selectioncol[0] = 1;
selectioncol[1] = 0;
selectioncol[2] = 0;
selectioncol[3] = 1;
#endif
}


/**
 * Draw the widget.
 *
 * This method mainly contains the original code from the base class,
 * but when drawing a highlighted list entry, the special selectcol[]
 * is used.
 */
void puaCRRCListBox::draw ( int dx, int dy )
{
  if ( !visible || ( window != puGetWindow () ) || list == NULL ) return ;

  abox.draw ( dx, dy, style, colour, isReturnDefault(), border_thickness ) ;

  /* If greyed out then halve the opacity when drawing the text */

  if ( active )
    glColor4fv ( colour [ PUCOL_LEGEND ] ) ;
  else
    glColor4f ( colour [ PUCOL_LEGEND ][0],
                colour [ PUCOL_LEGEND ][1],
                colour [ PUCOL_LEGEND ][2],
                colour [ PUCOL_LEGEND ][3] / 2.0f ) ; /* 50% more transparent */

  int xsize = abox.max[0] - abox.min[0] + 1 ;

  if ( r_cb )
    r_cb ( this, dx, dy, render_data ) ;
  else
  {
    int yinc = legendFont.getStringHeight () + PUSTR_BGAP ;
    int num_vis = getNumVisible () ;

    int selected ;
    getValue ( &selected ) ;

    for ( int i = top ; i < num && i < top + num_vis ; i++ )
    {
      if ( i == selected )
        glColor4f ( selectioncol[0],
                    selectioncol[1],
                    selectioncol[2],
                    selectioncol[3] );
      else
        glColor4f ( colour [ PUCOL_LABEL ][0],
                    colour [ PUCOL_LABEL ][1],
                    colour [ PUCOL_LABEL ][2],
                    colour [ PUCOL_LABEL ][3] ) ;

      int x = PUSTR_LGAP ;
      int y = yinc * ((i-top)+1) ;

      int xx = dx + abox.min[0] + x ;
      int yy = dy + abox.max[1] - y ;

      int width ;
      char str [ PUSTRING_MAX ] ;
      strcpy ( str, list [ i ] ) ;

      /*
        Does the string fit into the box?

        If not, chop it down one character at a time until
        it does fit.
      */

      while ( 1 )
      {
        width = legendFont.getStringWidth ( (char *)str ) + PUSTR_LGAP ;

        if ( width < xsize )
          break ;

        /*
          Nibble off one character and try again
          (Do that sneakily by replacing the last 4 characters with 3 dots)
        */

        strcpy ( & str [ strlen(str) - 4 ], "..." ) ;
      }

      legendFont.drawString ( (char*)str, xx, yy ) ;
    }
  }

  draw_label ( dx, dy ) ;
}




/**
 * Static function: handle slider movements.
 */
static void
handle_slider (puObject * slider)
{
  puListBox * box = (puListBox *)slider->getUserData();
  int index = int(box->getNumItems() * (1.0 - slider->getFloatValue()));
  if (index >= box->getNumItems())
    index = box->getNumItems() - 1;
  box->setTopItem(index);
}


/**
 * Static function: handle list entry selection.
 */
static void
handle_list_entry (puObject * listbox)
{
  puaScrListBox * box = (puaScrListBox *)listbox->getUserData();
  box->invokeCallback();
}


/**
 * Static function: handle arrow clicks.
 */
static void
handle_arrow (puObject * arrow)
{
  puSlider * slider = (puSlider *)arrow->getUserData();
  puListBox * list_box = (puListBox *)slider->getUserData();

  int step;
  switch (((puArrowButton *)arrow)->getArrowType())
  {
    case PUARROW_DOWN:
      step = 1;
      break;
    case PUARROW_UP:
      step = -1;
      break;
    default:
      step = 0;
      break;
  }

  int index = list_box->getTopItem();
  index += step;
  if (index < 0)
    index = 0;
  else if (index >= list_box->getNumItems())
    index = list_box->getNumItems() - 1;
  list_box->setTopItem(index);

  slider->setValue(1.0f - float(index)/list_box->getNumItems());
}

/** \brief Create an empty list box.
 *
 *  \param x X-coordinate of lower left corner.
 *  \param y Y-coordinate of lower left corner.
 *  \param w Widget width
 *  \param h Widget height
 */
puaScrListBox::puaScrListBox (int x, int y, int w, int h)
    : puGroup(x, y)
{
  init(w, h);
}


/** \brief Create a filled list box.
 *
 *  \param x X-coordinate of lower left corner.
 *  \param y Y-coordinate of lower left corner.
 *  \param w Widget width
 *  \param h Widget height
 *  \param contents Pointer to an array of pointers to the string items.
 */
puaScrListBox::puaScrListBox (int x, int y, int w, int h, char ** contents)
    : puGroup(x, y)
{
  init(w, h);
  newList(contents);
}


/**
 *  Destroy the ScrListBox
 */
puaScrListBox::~puaScrListBox ()
{
  puDeleteObject(_list_box);
  puDeleteObject(_slider);
  puDeleteObject(_up_arrow);
  puDeleteObject(_down_arrow);
}

/**
 *  Fill the list box with the strings from the given
 *  array of character pointers.
 */
void
puaScrListBox::newList (char ** contents)
{
  _list_box->newList(contents);
  _contents = contents;
}

/**
 *  Return a pointer to a character string with the
 *  contents of the selected entry.
 *  \return Pointer to highlighted entry or NULL if none is selected.
 */
char *
puaScrListBox::getStringValue ()
{
  int currElem = _list_box->getIntegerValue();
  if (currElem < 0)
  {
    return NULL;
  }
  else
  {
    return _contents[currElem];
  }
}

/**
 *  Set a pointer to a character string with the
 *  contents of the selected entry
 */
void
puaScrListBox::getValue(char **ps)
{
  int currElem = _list_box->getIntegerValue();
  if (currElem < 0)
  {
    *ps = NULL;
  }
  else
  {
    *ps = _contents[currElem];
  }
}


/**
 *  Set the integer pointed to by <code>i</code> to 
 *  the index of the currently selected entry.
 */
void
puaScrListBox::getValue(int *i)
{
  *i = _list_box->getIntegerValue();
}

/**
 *  Return the index of the currently selected entry.
 */
int
puaScrListBox::getIntegerValue()
{
  return _list_box->getIntegerValue();
}

/**
 *  Setup the widget.
 */
void
puaScrListBox::init (int w, int h)
{
  //~ _frame = new puFrame(0, 0, w, h);

  _list_box = new puaCRRCListBox(0, 0, w-17, h);
  
  //~ _list_box->setColour (PUCOL_BACKGROUND,1,0,0,1);//for better visibility of selected item on Plib-1.8.5
  //_list_box->setColour (PUCOL_FOREGROUND,.75,.75,.75,1);//darker for better visibility of selected item on Plib-1.8.4
 // _list_box->setStyle(-PUSTYLE_SMALL_SHADED);
  //~_list_box->setStyle(PUSTYLE_BOXED);//for not use PUCOL_BACKGROUND on outline
  _list_box->setUserData(this);
  _list_box->setCallback(handle_list_entry);
  _list_box->setValue(0);

  _slider = new puSlider(w-18, 18, h-34, true, 18);
  _slider->setValue(1.0f);
  _slider->setUserData(_list_box);
  _slider->setCallback(handle_slider);
  _slider->setCBMode(PUSLIDER_ALWAYS);

  _down_arrow = new puArrowButton(w-18, 0, w, 18, PUARROW_DOWN) ;
  _down_arrow->setUserData(_slider);
  _down_arrow->setCallback(handle_arrow);

  _up_arrow = new puArrowButton(w-18, h-18, w, h, PUARROW_UP);
  _up_arrow->setUserData(_slider);
  _up_arrow->setCallback(handle_arrow);

  close();
}
//~ void
//~ puaScrListBox::init (int w, int h)
//~ {
//~ _frame = new puFrame(0, 0, w, h);

//~ _list_box = new puListBox(0, 0, w-30, h);
//~ _list_box->setStyle(-PUSTYLE_SMALL_SHADED);
//~ _list_box->setUserData(this);
//~ _list_box->setValue(0);

//~ _slider = new puSlider(w-30, 30, h-60, true);
//~ _slider->setValue(1.0f);
//~ _slider->setUserData(_list_box);
//~ _slider->setCallback(handle_slider);
//~ _slider->setCBMode(PUSLIDER_ALWAYS);

//~ _down_arrow = new puArrowButton(w-30, 0, w, 30, PUARROW_DOWN) ;
//~ _down_arrow->setUserData(_slider);
//~ _down_arrow->setCallback(handle_arrow);

//~ _up_arrow = new puArrowButton(w-30, h-30, w, h, PUARROW_UP);
//~ _up_arrow->setUserData(_slider);
//~ _up_arrow->setCallback(handle_arrow);

//~ close();
//~ }

/**
 *  Set the widget's style.
 *  \param style A style as defined in pu.h (PUSTYLE_xxxx)
 */
void
puaScrListBox::setStyle(int style)
{
  _list_box->setStyle(style);
  _slider->setStyle(style);
  _down_arrow->setStyle(style);
  _up_arrow->setStyle(style);
  _style = style;
}


/**
 *  Return the number of items in the list
 */
int
puaScrListBox::getNumItems() const
{
  return _list_box->getNumItems();
}

/**
 *  Return the index of the top item that is displayed in the list
 */
int
puaScrListBox::getTopItem() const
{
  return _list_box->getTopItem();
}

/**
 *  Set the index of the top item to be displayed
 */
void
puaScrListBox::setTopItem(int item_index)
{
  _list_box->setTopItem(item_index);
  float new_val = 1.0f - (float)item_index/_list_box->getNumItems();
  _slider->setValue(new_val);
}

/**
 *  Set the item to be selected
 */
void
puaScrListBox::setValue(int i)
{
  _list_box->setValue(i);
}

// end of puaScrListBox.cpp

