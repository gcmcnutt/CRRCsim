/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005-2010 Jan Reucker (original author)
 * Copyright (C) 2006, 2008 Jens Wilhelm Wulf
 * Copyright (C) 2008, 2009, 2012 Joel Lienard
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
  

// implementation of class CGUILocationDialog
#include "../i18n.h"
#include "../global.h"
#include "../crrc_main.h"
#include "../global_video.h"
#include "../SimStateHandler.h"
#include "../mod_landscape/crrc_scenery.h"
#include "../mod_mode/F3A/handlerF3A.h"
#include "../mod_mode/F3F/handlerF3F.h"
#include "../mod_windfield/windfield.h"
#include "../mod_misc/filesystools.h"
#include "util.h"
#include "crrc_gui_main.h"
#include "crrc_location.h"

#include <iostream>
#include <sstream>
#include <sys/types.h>
#include <dirent.h>
#include <string>
using namespace std;



#define LIST_WIDGET_HEIGHT  (220)
#define LIST_WIDGET_WIDTH   (200)
#define BUTTON_BOX_HEIGHT   (2*DLG_DEF_SPACE+DLG_DEF_BUTTON_HEIGHT)
#define CONF_SEL_WIDTH      (256)
#define DESCRIPTION_HEIGHT  (100)
#define PREVIEW_HEIGHT      (128)
#define PREVIEW_WIDTH       (256)


static void CGUILocationCallback(puObject *obj);
static void CGUILocationSelectCallback(puObject *obj);


CGUILocationDialog::CGUILocationDialog() 
            : CRRCDialog(), cbox(NULL), preview_texture(NULL)
{
  // gather information on all installed sceneries
  DIR *dir=NULL;
  struct dirent *ent;
  const char* curLocName = Global::scenery->getName();
  int curSkyVariant = Global::scenery->getSkyVariant();
  int curLocIndex = -1;
  std::vector<std::string> paths;
  std::vector<std::string> extlist;
  extlist.push_back("xml");
  
  T_Config::getLocationDirs(paths);
  //std::cout << "Scanning scenery directories:" << std::endl;
  
  for (unsigned int i = 0; i < paths.size(); i++)
  {
    if ((dir = opendir(paths[i].c_str())) == NULL)
    {
      #ifdef DEBUG_GUI
      std::cerr << "createFileList(): unable to open directory " << paths[i];
      std::cerr << std::endl; 
      #endif
    }
    else
    {
     while ((ent = readdir(dir)) != NULL)
      {
        std::string tmp;
        bool        fMatch = false;
        
        tmp = ent->d_name;
        
        for (unsigned int n=0; n<extlist.size() && fMatch == false; n++)
        {
          if (T_GUI_Util::checkExtension(tmp, extlist[n]))
            fMatch = true;
        }
                   
        if (fMatch)
        {
          std::string fullpath = paths[i] + "/" + tmp;
          //std::cout << fullpath << std::endl;
          SimpleXMLTransfer *loc = NULL;
          bool ok=false;
          std::string name;
          try
          {
            loc = new SimpleXMLTransfer(fullpath);
            name = loc->getChild("name")->getContentString();
            ok=true;
          }
          catch (XMLException e)
          {
            std::cerr << "Caught XML exception in CGUIlocationSelectDialog:" << std::endl;
            std::cerr << "  " << e.what() << std::endl;
            std::cerr << "  File: " << fullpath << std::endl;
          }
          if(ok)
          {
            //std::cout << "  " << name << "(" << fullpath << ")" << std::endl;
            lists_insert(name, fullpath);
          }
          delete loc;
        }
      }
      closedir(dir);
    }
  }

  if (fileslist.size() == 0)
  {
    std::cerr << "  No sceneries found! Something is wrong with your installation!" << std::endl;
    std::cerr << "  Search path was:" << std::endl;
    for (unsigned int i = 0; i < paths.size(); i++)
    {
      std::cerr << "    " << paths[i] << std::endl;
    }
  }
  curLocIndex = index_in_locationslist(curLocName);
  filesList = T_GUI_Util::loadnames(fileslist, filesListSize);
  locationsList = T_GUI_Util::loadnames(locationslist, locationsListSize);
  skiesList = NULL;
  skiesListSize = 0;

  // Now setup the GUI widgets
  // height of a text label
  int msg_height  = puGetDefaultLegendFont().getStringHeight("jX")
                  + puGetDefaultLegendFont().getStringDescender()
                  + PUSTR_TGAP + PUSTR_BGAP;
  
  // top of the list box
  int top_of_listbox = BUTTON_BOX_HEIGHT 
                        + 2*DLG_DEF_SPACE 
                        + msg_height
                        + LIST_WIDGET_HEIGHT
                        + DESCRIPTION_HEIGHT;

  // Scrolled ListBox for the scenery files
  cbox = new puaScrListBox (DLG_DEF_SPACE,
                            top_of_listbox - LIST_WIDGET_HEIGHT,
                            LIST_WIDGET_WIDTH,
                            LIST_WIDGET_HEIGHT, 
                            locationsList);

  cbox->setLabelPlace(PUPLACE_TOP_LEFT);
  cbox->setLabel(_("Select location:"));
  cbox->setCallback(CGUILocationSelectCallback);
  cbox->setUserData(this);
  cbox->setValue(curLocIndex);

  ptext = new puText(DLG_DEF_SPACE, 
                     BUTTON_BOX_HEIGHT + DLG_DEF_SPACE + DESCRIPTION_HEIGHT);
  ptext->setLabel("File: /path/to/my/scenery.xml");

  
  // Combo box for sky variant selection
  sbox = new puaComboBox(2*DLG_DEF_SPACE + LIST_WIDGET_WIDTH,
                        top_of_listbox - DLG_DEF_BUTTON_HEIGHT,
                        2*DLG_DEF_SPACE + LIST_WIDGET_WIDTH + CONF_SEL_WIDTH,
                        top_of_listbox,
                        NULL,
                        false);
  sbox->setChildColourScheme(PUCLASS_POPUPMENU, dlgCol1[0], dlgCol1[1], dlgCol1[2]);
  sbox->setLabelPlace(PUPLACE_TOP_LEFT);
  sbox->setLabel(_("Select sky or resolution:"));
  sbox->setUserData(this);
  sbox->setCallback(CGUISkySelectCallback);
  
  // preview widget
  preview = new puaImageFrame (
          2*DLG_DEF_SPACE + LIST_WIDGET_WIDTH,
          top_of_listbox - DLG_DEF_BUTTON_HEIGHT -DLG_DEF_SPACE-PREVIEW_HEIGHT - DLG_DEF_BUTTON_HEIGHT, // lower left
          2*DLG_DEF_SPACE + LIST_WIDGET_WIDTH + PREVIEW_WIDTH,
          top_of_listbox - DLG_DEF_BUTTON_HEIGHT -DLG_DEF_SPACE - DLG_DEF_BUTTON_HEIGHT // upper right
          );
  /*preview->setLabelPlace(PUPLACE_TOP_LEFT);
  preview->setLabel("Preview:");*/

  // the description box
  description = new puaLargeInput( DLG_DEF_SPACE,
                                  BUTTON_BOX_HEIGHT + DLG_DEF_SPACE,
                                  2*DLG_DEF_SPACE + LIST_WIDGET_WIDTH + CONF_SEL_WIDTH,
                                  DESCRIPTION_HEIGHT,
                                  1,    // num of arrow pairs
                                  16,   // slider width
                                  1);   // wrap text
  description->disableInput();
  description->setText("This is a short description of the selected scenery.");
  /*description->setLabelPlace(PUPLACE_TOP_LEFT);
  description->setLabel(_("Description:"));*/

  // finalize the dialog
  setUserData(this);
  updateLocationInfo(-1);
  sbox->setCurrentItem(curSkyVariant);

  close();
  setSize(LIST_WIDGET_WIDTH + CONF_SEL_WIDTH + 4*DLG_DEF_SPACE, 
          BUTTON_BOX_HEIGHT + LIST_WIDGET_HEIGHT + msg_height + 5*DLG_DEF_SPACE + DESCRIPTION_HEIGHT );
  setCallback(CGUILocationCallback);
  centerOnScreen();
  reveal();
}

/**
 * Destroy the dialog.
 */
CGUILocationDialog::~CGUILocationDialog()
{
  T_GUI_Util::freenames(locationsList, locationsListSize);
  T_GUI_Util::freenames(filesList, filesListSize);
  T_GUI_Util::freenames(skiesList, skiesListSize);
  delete preview_texture;
}

/**
 * Update all widgets that depend on the currently selected location.
 */
void CGUILocationDialog::updateLocationInfo(int dlg_sky_index)
{
  std::vector<std::string> sky_descriptions;
  std::vector<std::string> sky_previews;//specifics previews for sky variant
  std::string preview_filename="";//generic preview for scenery
  int nLocId = getLocationId();
  std::string location_filename = "";
  if ((nLocId >= 0) && (nLocId < filesListSize))
  {
    location_filename = filesList[nLocId];
    SimpleXMLTransfer *xml = new SimpleXMLTransfer(location_filename);
    path_string = _("File: ");
    path_string += location_filename;
    ptext->setLabel(path_string.c_str());
    int children = xml->getChildCount();
    std::vector<int> sky_indices;
    for (int i = 0; i < children; i++)
    {
      SimpleXMLTransfer *child = xml->getChildAt(i);
      if (child->getName() == "sky")
      {
        sky_indices.push_back(i);
        std::string desc = child->getChild("descr_short.en", true)->getContentString();
        if (desc == "")
        {
          std::ostringstream s;
          s << sky_indices.size() << ": no description";
          desc = s.str();
        }
        sky_descriptions.push_back(desc);
        std::string prev = child->getChild("preview", true)->attribute("filename", "");
        sky_previews.push_back(prev);
        //std::cout << "  <sky> " << sky_indices.size() << " at child idx " << i << ": " << desc<< std::endl;
      }
      else if(child->getName() == "preview")
      {
        preview_filename =child->attribute("filename", "");
      }
    }

    // Update the description box
    description_string = "";
    try
    {
      description_string = xml->getChild("description.en")->getContentString();
    }
    catch (XMLException &e)
    {
      description_string = _("No description available.");
    }
    description_string = T_GUI_Util::cleanText(description_string);
    description->setText(description_string.c_str());
    description->setTopLineInWindow(0);
    description->setSelectRegion (0,0) ;
    delete xml;
  }
  else
  {
    std::cerr << "CGUILocationDialog::updateLocationInfo: index out of range:" << std::endl;
    std::cerr << "nLocId: " << nLocId << "   filesListSize: " << filesListSize << std::endl;
    return;
  }
  
  // If the scenery offers more than one sky to choose from,
  // update the combo box. Else hide it.
  if(dlg_sky_index == -1)//if not valide index, update combo box
  { 
    int sky_variant = 0;
    if ( sky_descriptions.size() > 1)
    {
      // Is there a skyvariant registered for this location
      SimpleXMLTransfer* ptr = cfg->getLocCfgPtr(cfgfile, location_filename);
      try
      {
        sky_variant = ptr->getChild("sky",false)->attributeAsInt("nUse",0);
        dlg_sky_index = sky_variant ;
      }
      catch (XMLException &e)
      {
        sky_variant = 0;
      }
      T_GUI_Util::freenames(skiesList, skiesListSize);
      skiesList = T_GUI_Util::loadnames( sky_descriptions, skiesListSize);
      sbox->newList(skiesList);
      sbox->reveal();
    }
    else
    {
      sbox->hide();
    }
    sbox->setCurrentItem(sky_variant);
  }
  
  // Update preview
  {
    std::string filename;
    if (dlg_sky_index < 0) dlg_sky_index = 0;
    if ((int)sky_previews.size() >= (dlg_sky_index+1))
      filename = sky_previews[dlg_sky_index];//specific preview  of sky variant
    if (filename == "") filename = preview_filename;//generic preview  of scenery     
    if (filename == "")
    {
      preview->hide();
    }
    else
    {
      if (preview_texture) delete(preview_texture);
      std::string filename1 = FileSysTools::getDataPath(filename);
      preview_texture = new ssgTexture(filename1.c_str());
      preview->setTexture(preview_texture);
      preview->reveal();
    }
  }
}

/******
    insert element in listes of location name and filename 
    with respect of alpabetic order of location name. 
********/
void CGUILocationDialog::lists_insert(std::string lname, std::string fullpath)
{
  vector<std::string>::iterator itf, itl;
  for (itf=fileslist.begin(), itl=locationslist.begin(); itl<locationslist.end(); itf++, itl++)
  {
    if(lname < *itl )
    {
    locationslist.insert(itl, lname);
    fileslist.insert(itf, fullpath);
    return;
    }
  }
  locationslist.insert(itl, lname);
  fileslist.insert(itf, fullpath);
}

/***************************/
int CGUILocationDialog::index_in_locationslist(std::string lname)
{
  vector<std::string>::iterator itl;
  int index = 0;
  for (itl=locationslist.begin(); itl<locationslist.end(); itl++)
  {
    if(lname == *itl )
    {
       return index;
    }
    index ++;
  }
  return -1;//not found
}

std::string CGUILocationDialog::getLocation() const
{
  std::string loc = cbox->getStringValue();
  return loc;
}

int CGUILocationDialog::getLocationId() const
{
  return cbox->getIntegerValue();
}

bool CGUILocationDialog::saveSelection() const
{
  const char* curLocName = Global::scenery->getName();
  int curSkyVariant = Global::scenery->getSkyVariant();
  int id = getLocationId();
  std::string filename = filesList[id];
  std::string newLocname = locationsList[id];
  int newSkyVariant = sbox->getCurrentItem();
  if (newSkyVariant < 0)
  {
    // If the combo box was empty, the current item was -1.
    // In this case, correct the sky variant index to 0.
    newSkyVariant = 0;
  }

  if ((newLocname != curLocName) || (newSkyVariant != curSkyVariant))
  {
    // temporarily set a null scenery to erase 3D display
    Scenery* cur_scenery = Global::scenery;
    Global::scenery = new SceneryNull();
    
    Scenery* new_scenery = loadScenery(FileSysTools::getDataPath(filename).c_str(),
                                       newSkyVariant);

    // restore original scenery
    delete Global::scenery;
    Global::scenery = cur_scenery;

    if (new_scenery)
    {
      cfg->setLocation(filename.c_str(), newSkyVariant, cfgfile);
      
      clear_wind_field();
      delete Global::scenery;
      Global::scenery = new_scenery;
      
      //reinitialise game mode
      if (Global::gameHandler)
      {
        delete Global::gameHandler;
      }
      if (cfgfile->getInt("game.f3a.enabled",0))
        Global::gameHandler = new HandlerF3A();
      else if (cfgfile->getInt("game.f3f.enabled",0))
        Global::gameHandler = new HandlerF3F();
      else
        Global::gameHandler= new T_GameHandler();
    }
    cfg->read(cfgfile);
    Video::setWindowTitleString();
    player_pos = Global::scenery->getPlayerPosition();
    Global::Simulation->reset();
  }
  return true;
}

/** \brief The dialog's callback.
 *
 *  Load the new location.
 */
void CGUILocationCallback(puObject *obj)
{
  if (obj->getIntegerValue() == CRRC_DIALOG_OK)
  {
    CGUILocationDialog *dlg = (CGUILocationDialog*)obj->getUserData();
    dlg->hide();
    dlg->saveSelection();
  }

  Global::gui->hide();
  puDeleteObject(obj);
}

/** \brief The location list's callback.
 *
 *  Update scenery information whenever a new
 *  location is chosen.
 */
void CGUILocationSelectCallback(puObject *obj)
{
  CGUILocationDialog* dlg = static_cast<CGUILocationDialog*>(obj->getUserData());
  dlg->updateLocationInfo(-1);
}

void CGUILocationDialog::CGUISkySelectCallback(puObject *obj)
{
  CGUILocationDialog *dlg = (CGUILocationDialog*)obj->getUserData();
  int newSkyVariant = dlg->sbox->getCurrentItem();
  if (newSkyVariant < 0) newSkyVariant = 0;
  dlg->updateLocationInfo(newSkyVariant);
}
