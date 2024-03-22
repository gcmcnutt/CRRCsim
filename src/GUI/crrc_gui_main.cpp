/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2004-2009 Jan Reucker (original author)
 * Copyright (C) 2005-2008, 2010 Jens Wilhelm Wulf
 * Copyright (C) 2005, 2008 Olivier Bordes
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
  

/** \file crrc_gui_main.cpp
 *
 *  The central file responsible for managing the graphical user interface.
 */

#include "../i18n.h"
#include "../global.h"
#include "../SimStateHandler.h"
#include "crrc_gui_main.h"
#include "crrc_gui_viewwind.h"
#include "crrc_dialog.h"
#include "crrc_msgbox.h"
#include "puaImageFrame.h"
#include "crrc_planesel.h"
#include "crrc_video.h"
#include "crrc_launch.h"
#include "crrc_windthermal.h"
#include "crrc_joy.h"
#include "crrc_calibmap.h"
#include "../crrc_main.h"
#include "../config.h"
#include "../zoom.h"
#include "../crrc_sound.h"
#include "crrc_ctrlgen.h"
#include "crrc_audio.h"
#include "crrc_location.h"
#include "crrc_f3a.h"
#include "crrc_f3f.h"
#include "crrc_loadrobot.h"
#include "crrc_setrecordname.h"
#include "../robots.h"
#include "../mod_misc/filesystools.h"
#include "../mod_misc/lib_conversions.h"
#include "../mod_video/crrc_graphics.h"
#include "../mod_video/fonts.h"


#ifdef linux
#include "../global_video.h"
#endif

//void GUI_IdleFunction(TSimInputs *in);

static void quitDialogCallback(int);

// Callback prototypes
static void file_exit_cb(puObject*);
static void file_save_cb(puObject*);
static void file_set_record_cb(puObject*);

static void view_wind_cb(puObject*);
static void view_fullsc_cb(puObject*);
static void view_train_cb(puObject*);
static void view_testmode_cb(puObject*);
static void view_hudcompass_cb(puObject*);
static void view_windvectors_cb(puObject*);
static void view_verbosity_cb(puObject*);
static void view_modelwindow_cb(puObject*);
static void view_zoomin_cb(puObject*);
static void view_zoomout_cb(puObject*);
static void view_unzoom_cb(puObject*);

static void sim_restart_cb(puObject*);

static void opt_plane_cb(puObject*);
static void opt_launch_cb(puObject*);
static void opt_location_cb(puObject*);
static void opt_windthermal_cb(puObject*);
static void opt_video_cb(puObject*);
static void opt_audio_cb(puObject*);
static void opt_ctrl_general_cb(puObject*);

static void game_f3a_cb(puObject*);
static void game_f3f_cb(puObject*);

static void help_web_cb(puObject*);
static void help_keys_cb(puObject*);
static void help_about_cb(puObject*);

static void robot_load_cb(puObject*);
static void robot_rm_all_cb(puObject*);
                                       

/** \brief Create the GUI object.
 *
 *  Creates the GUI and sets its "visible" state.
 *  \param vis Set "visible" state. Defaults to true.
 */
CGUIMain::CGUIMain(bool vis) : visible(vis)
{
  fntInit();
  puInit();
  puSetDefaultStyle(PUSTYLE_SMALL_BEVELLED);

  // Light grey, no transparency
  puSetDefaultColourScheme(0.85, 0.85, 0.85, 1.0);
  
  // Menu entries and callback mapping
  // Caution: submenu-entries must be declared in reverse order!
  
  // File menu
  const char *file_submenu[]    = {_("Exit"), _("Save this Flight's Log as.."),_("Save Settings"), NULL};
  static puCallback file_submenu_cb[]  = {file_exit_cb, file_set_record_cb, file_save_cb, NULL};

  // View submenu
  const char *view_submenu[]   = { _("Inspect Wind"),
                                   _("Reset Zoom"),_("Zoom -"), 
                                   _("Zoom +"), _("Toggle Verbosity"),
                                   _("Toggle Wind vectors"),
                                   _("Toggle HUD Compass"),
                                   _("Toggle Test Mode"),
                                   _("Toggle Training Mode"),
                                   _("Toggle Model Window Mode"),
                                   _("Toggle Fullscreen"),
                                   NULL};
  puCallback view_submenu_cb[] = {  view_wind_cb,
                                    view_unzoom_cb, view_zoomout_cb,
                                    view_zoomin_cb, view_verbosity_cb,
                                    view_windvectors_cb,
                                    view_hudcompass_cb,
                                    view_testmode_cb,
                                    view_train_cb,
                                    view_modelwindow_cb,                     
                                    view_fullsc_cb,
                                    NULL};
  // Simulation menu
  const char *sim_submenu[]   = {_("Restart"), NULL};
  puCallback sim_submenu_cb[] = {sim_restart_cb, NULL};

  // Options menu
  const char *opt_submenu[]  = {_("Audio"),
                                _("Controls"),
                                _("Video"), _("Wind, Thermals"), 
                                _("Launch"), _("Location"),
                                _("Airplane"), NULL};
  puCallback opt_submenu_cb[] = { opt_audio_cb,
                                  opt_ctrl_general_cb,
                                  opt_video_cb, 
                                  opt_windthermal_cb, opt_launch_cb, 
                                  opt_location_cb, opt_plane_cb, NULL};

  // Game menu
  const char *game_submenu[]   = {"F3A", "F3F", NULL};
  static puCallback game_submenu_cb[] = {game_f3a_cb, game_f3f_cb, NULL};

  // Help menu
  const char *help_submenu[]   = {_("About"), _("Keys"), _("Help"), NULL};
  puCallback help_submenu_cb[] = {help_about_cb, help_keys_cb, help_web_cb, NULL};

  // Robots menu
  const char *robot_submenu[]   = {_("Remove all Robots"), _("Load Robot"), NULL};
  puCallback robot_submenu_cb[] = {robot_rm_all_cb, robot_load_cb, NULL};

  // create the menu bar
  main_menu_bar = new puMenuBar() ;
  main_menu_bar->add_submenu(_("File"), (char**)file_submenu, file_submenu_cb);
  main_menu_bar->add_submenu(_("View"), (char**)view_submenu, view_submenu_cb);
  main_menu_bar->add_submenu(_("Simulation"), (char**)sim_submenu, sim_submenu_cb);
  main_menu_bar->add_submenu(_("Options"), (char**)opt_submenu, opt_submenu_cb);
  main_menu_bar->add_submenu(_("Game"), (char**)game_submenu, game_submenu_cb);
  main_menu_bar->add_submenu(_("Robots"), (char**)robot_submenu, robot_submenu_cb);
  main_menu_bar->add_submenu(_("Help"), (char**)help_submenu, help_submenu_cb);
  main_menu_bar->close();

  // create the verbosity display
  verboseOutput = new puText(30, 30);
  verboseOutput->setColour(PUCOL_LABEL, 1, 0.1, 0.1);

  if (Video::textureFont)
  {
    verboseOutput->setLabelFont(Video::textureFont);
  }
  verboseOutput->reveal();

  // create the text widgets for HUD compass
  for( int i = 0; i <= nCompass; i++ )
  {
    compass_x[i] = new puText(0, 0);
    compass_x[i]->setColour(PUCOL_LABEL, 1., 0.1, 0.1, 0.7);
    compass_x[i]->setLabelPlace(PUPLACE_BOTTOM_CENTERED);
    if (Video::window_xsize > 800)
      //compass_x[i]->setLabelFont(PUFONT_HELVETICA_18); 
    compass_x[i]->setLabel(""); 
    compass_x[i]->hide();
  }
  for( int i = 0; i <= nCompass; i++ )
  {
    compass_y[i] = new puText(0, 0);
    compass_y[i]->setColour(PUCOL_LABEL, 1., 0.1, 0.1, 0.7);
    compass_y[i]->setLabelPlace(PUPLACE_CENTERED_RIGHT); 
    if (Video::window_xsize > 800)
      //compass_y[i]->setLabelFont(PUFONT_HELVETICA_18); 
    compass_y[i]->setLabel(""); 
    compass_y[i]->hide();
  }

  // show or hide the GUI
  if (visible)
  {
    main_menu_bar->reveal();
    SDL_ShowCursor(SDL_ENABLE);
  }
  else
  {
    main_menu_bar->hide();
    if (Global::TXInterface->inputMethod() == T_TX_Interface::eIM_mouse)
      SDL_ShowCursor(SDL_ENABLE);
    else
      SDL_ShowCursor(SDL_DISABLE);
  }
}


/** \brief Destroy the gui object.
 *
 *
 */
CGUIMain::~CGUIMain()
{
  puDeleteObject(main_menu_bar);
  main_menu_bar = NULL;
}


/** \brief Hide the GUI.
 *
 *  This method hides the GUI and all included widgets.
 */
void CGUIMain::hide()
{
  visible = false;
  Global::Simulation->resume();
  main_menu_bar->hide();
  if (Global::TXInterface->inputMethod() == T_TX_Interface::eIM_mouse)
    SDL_ShowCursor(SDL_ENABLE);
  else
    SDL_ShowCursor(SDL_DISABLE);
  //Global::Simulation->resetIdle();
}


/** \brief Show the GUI.
 *
 *  This method sets the GUIs "visible" state to true and
 *  activates all included widgets.
 */
void CGUIMain::reveal()
{
  //Global::Simulation->setNewIdle(GUI_IdleFunction);
  visible = true;
  Global::Simulation->pause();
  main_menu_bar->reveal();
  SDL_ShowCursor(SDL_ENABLE);
  LOG(_("Press <ESC> to hide menu and resume simulation."));
}


/** \brief Draw the GUI.
 *
 *  This method has to be called for each OpenGL frame as
 *  long as the GUI is visible. Note: to make the GUI
 *  invisible it's not sufficient to stop calling draw(),
 *  because any visible widget will remain clickable!
 *  Make sure to call hide() before, or simply call draw()
 *  as long as the GUI isVisible().
 */
void CGUIMain::draw()
{
  glPushAttrib(GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT);
  glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
  glAlphaFunc(GL_GREATER,0.1f);
  glEnable(GL_BLEND);
  puDisplay();
  glPopAttrib();
}


/** \brief The GUI's key press event handler.
 *
 *  This function should be called from the SDL event loop. It takes a keyboard event
 *  as an argument, translates it to PUI syntax and passes it to the
 *  PUI-internal keyboard function. If there's no active widget which could
 *  use the key event the function will return false, giving the caller
 *  the opportunity to use the event for other purposes.
 *  \param key A key symbol generated by an SDL keyboard event.
 *  \return true if PUI was able to handle the event
 */
bool CGUIMain::keyDownEventHandler(SDL_keysym& key)
{
  int tkey;
  bool ret;

  tkey = translateKey(key);
  ret = puKeyboard(tkey, PU_DOWN);
  
  // ESC key handling
  // note: translateKey() does not affect the ESC keysym,
  // so it is safe to test the SDL key value here
  if (!ret && (tkey == SDLK_ESCAPE))
  {
    if (isVisible())
    {
      CRRCDialog* top = CRRCDialog::getToplevel();
      if (top != NULL)
      {
        if (top->hasCancelButton())
        {
          //std::cout << "Invoking CANCEL for toplevel dialog" << std::endl;
          top->setValue(CRRC_DIALOG_CANCEL);
          top->invokeCallback();
        }
      }
      else
      {
        //std::cout << "No active dialog, hiding GUI" << std::endl;
        hide();
      }
    }
    else
    {
      reveal();
    }
    ret = true;
  }

  return ret;
}

/** \brief The GUI's key release event handler.
 *
 *  This function should be called from the SDL event loop. It takes a keyboard event
 *  as an argument, translates it to PUI syntax and passes it to the
 *  PUI-internal keyboard function. If there's no active widget which could
 *  use the key event the function will return false, giving the caller
 *  the opportunity to use the event for other purposes.
 *  \param key A key symbol generated by an SDL keyboard event.
 *  \return true if PUI was able to handle the event
 */
bool CGUIMain::keyUpEventHandler(SDL_keysym& key)
{
  int tkey = translateKey(key);
  return puKeyboard(tkey, PU_UP);
}

/** \brief The GUI's mouse button press event handler.
 *
 *  This function should be called from the SDL event loop.
 *  It translates a mouse event to PUI syntax and passes it to the
 *  PUI-internal mouse function. If there's no active widget which could
 *  use the event the function will return false, giving the caller
 *  the opportunity to use the event for other purposes.
 *  \param btn Code of the button as reported by SDL
 *  \param x Mouse x coordinate as reported by SDL
 *  \param y Mouse y coordinate as reported by SDL
 *  \return true if PUI was able to handle the event
 */
bool CGUIMain::mouseButtonDownHandler(int btn, int x, int y)
{
  return puMouse(translateMouse(btn), PU_DOWN, x, y);
}

/** \brief The GUI's mouse button release event handler.
 *
 *  This function should be called from the SDL event loop.
 *  It translates a mouse event to PUI syntax and passes it to the
 *  PUI-internal mouse function. If there's no active widget which could
 *  use the event the function will return false, giving the caller
 *  the opportunity to use the event for other purposes.
 *  \param btn Code of the button as reported by SDL
 *  \param x Mouse x coordinate as reported by SDL
 *  \param y Mouse y coordinate as reported by SDL
 *  \return true if PUI was able to handle the event
 */
bool CGUIMain::mouseButtonUpHandler(int btn, int x, int y)
{
  return puMouse(translateMouse(btn), PU_UP, x, y);
}


/** \brief The GUI's mouse motion handler.
 *
 *
 */
bool CGUIMain::mouseMotionHandler(int x, int y)
{
  return puMouse(x, y);
}


/** \brief Translate SDL key macros to PUI macros.
 *
 *  Make sure that SDL unicode support is turned on to
 *  make this work!
 */
int CGUIMain::translateKey(const SDL_keysym& keysym)
{
  // Printable characters
  if (keysym.unicode > 0)
    return keysym.unicode;

  // Numpad key, translate no non-numpad equivalent
  if (keysym.sym >= SDLK_KP0 && keysym.sym <= SDLK_KP_EQUALS) 
  {
    switch (keysym.sym) 
    {
      case SDLK_KP0:
        return PU_KEY_INSERT;
      case SDLK_KP1:
        return PU_KEY_END;
      case SDLK_KP2:
        return PU_KEY_DOWN;
      case SDLK_KP3:
        return PU_KEY_PAGE_DOWN;
      case SDLK_KP4:
        return PU_KEY_LEFT;
      case SDLK_KP6:
        return PU_KEY_RIGHT;
      case SDLK_KP7:
        return PU_KEY_HOME;
      case SDLK_KP8:
        return PU_KEY_UP;
      case SDLK_KP9:
        return PU_KEY_PAGE_UP;
      default:
        return -1;
    }
  }

  // Everything else
  switch (keysym.sym) 
  {
    case SDLK_UP:
      return PU_KEY_UP;
    case SDLK_DOWN:
      return PU_KEY_DOWN;
    case SDLK_LEFT:
      return PU_KEY_LEFT;
    case SDLK_RIGHT:
      return PU_KEY_RIGHT;
  
    case SDLK_PAGEUP:
      return PU_KEY_PAGE_UP;
    case SDLK_PAGEDOWN:
      return PU_KEY_PAGE_DOWN;
    case SDLK_HOME:
      return PU_KEY_HOME;
    case SDLK_END:
      return PU_KEY_END;
    case SDLK_INSERT:
      return PU_KEY_INSERT;
    case SDLK_DELETE:
      return -1;
  
    case SDLK_F1:
      return PU_KEY_F1;
    case SDLK_F2:
      return PU_KEY_F2;
    case SDLK_F3:
      return PU_KEY_F3;
    case SDLK_F4:
      return PU_KEY_F4;
    case SDLK_F5:
      return PU_KEY_F5;
    case SDLK_F6:
      return PU_KEY_F6;
    case SDLK_F7:
      return PU_KEY_F7;
    case SDLK_F8:
      return PU_KEY_F8;
    case SDLK_F9:
      return PU_KEY_F9;
    case SDLK_F10:
      return PU_KEY_F10;
    case SDLK_F11:
      return PU_KEY_F11;
    case SDLK_F12:
      return PU_KEY_F12;
  
    default:
      return -1;
  }
}

// description: see header file
void CGUIMain::setVerboseText(const char* msg)
{ 
  verboseOutput->setLabel(msg); 
};

// description: see header file
void CGUIMain::doHUDCompass(const float field_of_view)
{  
  if (Global::HUDCompass && !isVisible())
  {
    // compass visible..
    puFont stdFont, bigFont;
    if (Video::textureFont)
    {
      stdFont = puFont ( Video::textureFont, Video::window_xsize > 800 ? 20 : 14 );
      bigFont = puFont ( Video::textureFont, Video::window_xsize > 800 ? 24 : 18 );
    }
    else{    
      stdFont = Video::window_xsize > 800 ? PUFONT_HELVETICA_18 : PUFONT_9_BY_15; 
      bigFont = Video::window_xsize > 800 ? PUFONT_TIMES_ROMAN_24 : PUFONT_HELVETICA_18;
    }
    CRRCMath::Vector3 look_dir = Video::looking_pos - player_pos;
    float azimuth = atan2(look_dir.r[0], -look_dir.r[2])*180.0/M_PI;
    float elevation = atan2(look_dir.r[1], sqrt(pow(look_dir.r[0],2) + pow(look_dir.r[2],2)))*180.0/M_PI;
    azimuth = azimuth < 0.0 ? azimuth + 360.0 : azimuth;
    // for some reason the actual field of view is quite smaller than specified by field_of_view
    // and if not corrected this would result in inaccurate compass labels
    float fov_x = 0.66 * field_of_view * Video::window_xsize/Video::window_ysize;
    float fov_y = 0.66 * field_of_view;
    float kx = 1.0/atan(0.5*fov_x/180.0*M_PI);
    float ky = 1.0/atan(0.5*fov_y/180.0*M_PI);
    float step = (int)(fov_x/nCompass);
    if ( step < 1 )
      step = 1;
    else if ( step < 5 )
      step = 5;
    else if ( step < 15 )
      step = 15;
    else if ( step < 30 )
      step = 30;
    else if ( step < 45 )
      step = 45;
    else
      step = 90;
    
    int az0 = (int)(azimuth/step) * step;
    int el0 = (int)(elevation/step) * step;

    for( int i = 0; i <= nCompass; i++ )
    {
      int az = az0 + (i - nCompass/2)*step;
      int xx = Video::window_xsize/2*(1.0 + kx*atan((az - azimuth)/180.0*M_PI));
      az = az < 0 ? az + 360 : az;
      az = az >= 360 ? az - 360 : az;
      if (xx > 0 && xx < Video::window_xsize)
      {
        compass_x[i]->setLabelFont(bigFont); 
        compass_x[i]->setColour(PUCOL_LABEL, 1., 0.1, 0.1, 1.);
        switch (az)
        {
          case 0:
            compass_msg_x[i] = "N";
            break;
          case 45:
            compass_msg_x[i] = "NE";
            break;
          case 90:
            compass_msg_x[i] = "E";
            break;
          case 135:
            compass_msg_x[i] = "SE";
            break;
          case 180:
            compass_msg_x[i] = "S";
            break;
          case 225:
            compass_msg_x[i] = "SW";
            break;
          case 270:
            compass_msg_x[i] = "W";
            break;
          case 315:
            compass_msg_x[i] = "NW";
            break;
          default:
            compass_msg_x[i] = itoStr(az, '0', 3, true);
            compass_x[i]->setLabelFont(stdFont); 
            compass_x[i]->setColour(PUCOL_LABEL, 1., 0.1, 0.1, 0.7);
        }
        compass_x[i]->setPosition(xx, Video::window_ysize); 
        compass_x[i]->setLabel(compass_msg_x[i].c_str()); 
        compass_x[i]->reveal();
      }
      else
        compass_x[i]->hide();
    }

    for( int i = 0; i <= nCompass; i++ )
    {
      int el = el0 + (i - nCompass/2)*step;
      int yy = Video::window_ysize/2*(1.0 + ky*atan((el - elevation)/180.0*M_PI));
      if (yy > 0 && yy < Video::window_ysize)
      {
        if (el == 0)
        {
          compass_msg_y[i] = "0";
          compass_y[i]->setLabelFont(bigFont); 
          compass_y[i]->setColour(PUCOL_LABEL, 1., 0.1, 0.1, 1.);
        }
        else
        {
          compass_msg_y[i] = itoStr(abs(el), '0', 2, true);
          compass_msg_y[i] = (el > 0 ? '+' : '-') + compass_msg_y[i];
          compass_y[i]->setLabelFont(stdFont); 
          compass_y[i]->setColour(PUCOL_LABEL, 1., 0.1, 0.1, 0.7);
        }
        compass_y[i]->setPosition(0, yy); 
        compass_y[i]->setLabel(compass_msg_y[i].c_str()); 
        compass_y[i]->reveal();
      }
      else
        compass_y[i]->hide();
    }
    
    // ..compass visible
  }
  else
  {
    // compass hidden..
    for( int i = 0; i <= nCompass; i++ )
    {
      compass_x[i]->hide();
      compass_y[i]->hide();
    }
    // ..compass hidden
  }
};

// description: see header file
void CGUIMain::errorMsg(const char* message)
{
  fprintf(stderr, "--- GUI error popup ---\n%s\n--- end popup ---------\n", message);
  new CGUIMsgBox(message);
}

void CGUIMain::doQuitDialog()
{
  reveal();
  if (options_changed())
  {
    CGUIMsgBox *msg = new CGUIMsgBox(_("Configuration has changed, save?"), 
                                      CRRC_DIALOG_OK | CRRC_DIALOG_CANCEL,
                                      quitDialogCallback);
    msg->setOKButtonLegend(_("Yes"));
    msg->setCancelButtonLegend(_("No"));
  }
  else
  {
    Global::Simulation->quit();
  }
}

// The menu entry callbacks.

void optionNotImplementedYetBox()
{
  new CGUIMsgBox("Changing this from the GUI isn't implemented yet.\n"
                 "Please see crrcsim.xml and documentation/options.txt\n"
                 "for how to configure CRRCSim.");
}

static void file_exit_cb(puObject *obj)
{
  Global::gui->doQuitDialog();
}

static void file_save_cb(puObject *obj)
{
  options_saveToFile();
}

static void file_set_record_cb(puObject*)
{
  new CGUISetRecordNameDialog();  
}

static void sim_restart_cb(puObject *obj)
{
  Global::Simulation->reset();
  Global::gui->hide();
}

static void opt_plane_cb(puObject *obj)
{
  new CGUIPlaneSelectDialog();
}

static void opt_location_cb(puObject *obj)
{
  new CGUILocationDialog();
}

static void opt_launch_cb(puObject *obj)
{
  new CGUILaunchDialog();
}

static void opt_windthermal_cb(puObject *obj)
{
  new CGUIWindThermalDialog();
}

static void opt_video_cb(puObject *obj)
{
  new CGUIVideoDialog();
}

static void opt_audio_cb(puObject *obj)
{
  new CGUIAudioDialog();
}

static void opt_ctrl_general_cb(puObject *obj)
{
  new CGUICtrlGeneralDialog();
}

static void game_f3a_cb(puObject *obj)
{
  new CGUIF3ADialog();
}

static void game_f3f_cb(puObject *obj)
{
  new CGUIF3FDialog();
}

static void robot_load_cb(puObject *obj)
{
  new CGUILoadRobotDialog();
}

static void robot_rm_all_cb(puObject *obj)
{
  Global::robots->RemoveAll();
}

static void help_web_cb(puObject *obj)
{
  new CGUIMsgBox(_("See http://crrcsim.sourceforge.net/ for more information.\n\n"
                 "With your copy of CRRCSim you also received documentation\n"
                 "in a subdirectory named \"documentation\". Take a look at \"index.html\"."));
}

static void help_keys_cb(puObject *obj)
{
  const int keyw = 125;
  const int txtw = 500;
  int y_in_win = 1 * DLG_DEF_BUTTON_HEIGHT + 2 * DLG_DEF_SPACE;
  int dy = DLG_CHECK_H + 0.4 * DLG_DEF_SPACE;
  
  CRRCDialog *KeysDialog = new CRRCDialog(0, 0, CRRC_DIALOG_OK);

  puText *txta01 = new puText(DLG_DEF_SPACE, y_in_win);
  txta01->setLabel("-");
  puText *txtb01 = new puText(keyw + DLG_DEF_SPACE, y_in_win);
  txtb01->setLabel(_("zoom out (assuming zoom.control is KEYBOARD)"));

  puText *txta02 = new puText(DLG_DEF_SPACE, y_in_win += dy);
  txta02->setLabel("+");
  puText *txtb02 = new puText(keyw + DLG_DEF_SPACE, y_in_win);
  txtb02->setLabel(_("zoom in (assuming zoom.control is KEYBOARD)"));

  puText *txta03 = new puText(DLG_DEF_SPACE, y_in_win += dy);
  txta03->setLabel(_("up/down arrow"));
  puText *txtb03 = new puText(keyw + DLG_DEF_SPACE, y_in_win);
  txtb03->setLabel(_("elevator"));

  puText *txta04 = new puText(DLG_DEF_SPACE, y_in_win += dy);
  txta04->setLabel(_("left/right arrow"));
  puText *txtb04 = new puText(keyw + DLG_DEF_SPACE, y_in_win);
  txtb04->setLabel(_("rudder"));

  puText *txta05 = new puText(DLG_DEF_SPACE, y_in_win += dy);
  txta05->setLabel(_("pg-dwn"));
  puText *txtb05 = new puText(keyw + DLG_DEF_SPACE, y_in_win);
  txtb05->setLabel(_("decrease throttle (if you aren't using JOYSTICK or better)"));

  puText *txta06 = new puText(DLG_DEF_SPACE, y_in_win += dy);
  txta06->setLabel(_("pg-up"));
  puText *txtb06 = new puText(keyw + DLG_DEF_SPACE, y_in_win);
  txtb06->setLabel(_("increase throttle (if you aren't using JOYSTICK or better)"));

  puText *txta07 = new puText(DLG_DEF_SPACE, y_in_win += dy);
  txta07->setLabel("b");
  puText *txtb07 = new puText(keyw + DLG_DEF_SPACE, y_in_win);
  txtb07->setLabel(_("toggle spoiler/airbrake (if function is not mapped to a controller)"));

  puText *txta08 = new puText(DLG_DEF_SPACE, y_in_win += dy);
  txta08->setLabel("g");
  puText *txtb08 = new puText(keyw + DLG_DEF_SPACE, y_in_win);
  txtb08->setLabel(_("toggle landing gear (if function is not mapped to a controller)"));

  puText *txta09 = new puText(DLG_DEF_SPACE, y_in_win += dy);
  txta09->setLabel("w");
  puText *txtb09 = new puText(keyw + DLG_DEF_SPACE, y_in_win);
  txtb09->setLabel(_("toggle 3D wind vector visualisation mode (0..2)"));

  puText *txta10 = new puText(DLG_DEF_SPACE, y_in_win += dy);
  txta10->setLabel("h");
  puText *txtb10 = new puText(keyw + DLG_DEF_SPACE, y_in_win);
  txtb10->setLabel(_("toggle HUD compass visualisation mode"));

  puText *txta11 = new puText(DLG_DEF_SPACE, y_in_win += dy);
  txta11->setLabel("v");
  puText *txtb11 = new puText(keyw + DLG_DEF_SPACE, y_in_win);
  txtb11->setLabel(_("toggle verbosity level (0..3) to display control inputs/FOV/FPS"));

  puText *txta12 = new puText(DLG_DEF_SPACE, y_in_win += dy);
  txta12->setLabel("t");
  puText *txtb12 = new puText(keyw + DLG_DEF_SPACE, y_in_win);
  txtb12->setLabel(_("toggle training mode which displays the location of the thermals"));

  puText *txta13 = new puText(DLG_DEF_SPACE, y_in_win += dy);
  txta12->setLabel("m");
  puText *txtb13 = new puText(keyw + DLG_DEF_SPACE, y_in_win);
  txtb12->setLabel(_("toggle model's close-view window visualisation mode (0..2)"));

  puText *txta14 = new puText(DLG_DEF_SPACE, y_in_win += dy);
  txta13->setLabel("d");
  puText *txtb14 = new puText(keyw + DLG_DEF_SPACE, y_in_win);
  txtb13->setLabel(_("toggle control input debugging mode"));

  puText *txta15 = new puText(DLG_DEF_SPACE, y_in_win += dy);
  txta14->setLabel("c");
  puText *txtb15 = new puText(keyw + DLG_DEF_SPACE, y_in_win);
  txtb14->setLabel(_("reload model configuration"));

  puText *txta16 = new puText(DLG_DEF_SPACE, y_in_win += dy);
  txta15->setLabel("p");
  puText *txtb16 = new puText(keyw + DLG_DEF_SPACE, y_in_win);
  txtb15->setLabel(_("pause/resume simulation"));

  puText *txta17 = new puText(DLG_DEF_SPACE, y_in_win += dy);
  txta16->setLabel("r");
  puText *txtb17 = new puText(keyw + DLG_DEF_SPACE, y_in_win);
  txtb16->setLabel(_("restart after crash"));

  puText *txta18 = new puText(DLG_DEF_SPACE, y_in_win += dy);
  txta17->setLabel("q");
  puText *txtb18 = new puText(keyw + DLG_DEF_SPACE, y_in_win);
  txtb17->setLabel(_("quit"));

  puText* __attribute__((unused)) txta19 = new puText(DLG_DEF_SPACE, y_in_win += dy);
  txta18->setLabel("ESC");
  puText* __attribute__((unused)) txtb19 = new puText(keyw + DLG_DEF_SPACE, y_in_win);
  txtb18->setLabel(_("show/hide menu"));

  puText *txta00 = new puText(DLG_DEF_SPACE, y_in_win += dy + DLG_DEF_SPACE);
  txta00->setLabel(_("Key mapping:\n"));

  y_in_win += dy;
  

  KeysDialog->close(); 
  KeysDialog->setSize(1 * keyw + 1 * txtw + 3 * DLG_DEF_SPACE,
                      1 * y_in_win        + 1 * DLG_DEF_SPACE);
  KeysDialog->centerOnScreen();
  KeysDialog->reveal();  
}

static void help_about_cb(puObject *obj)
{
  const int txtw = 200;
  const int imgw = 128;
  const int imgh = 128;
  int y_in_win = 1 * DLG_DEF_BUTTON_HEIGHT + 2 * DLG_DEF_SPACE;
  static ssgTexture *image = NULL;
  
  CRRCDialog *AboutDialog = new CRRCDialog(0, 0, CRRC_DIALOG_OK);

  // package identification: string needs to be persistent after construction
  static std::string package_txt;
  package_txt = PACKAGE_NAME;
  package_txt += "\n\n";
  package_txt += _("version info:\n");
  package_txt += PACKAGE_VERSION;
  puText *txt01 = new puText(2 * DLG_DEF_SPACE + imgw, y_in_win + imgh);
  txt01->setLabel(package_txt.c_str());
  txt01->setLabelPlace(PUPLACE_BELOW_RIGHT);
  
  // load "About" image file
  if (!image)
  {
    std::string imageAboutFilename = FileSysTools::getDataPath("textures/Crrcsim.jpg");
    image = new ssgTexture(imageAboutFilename.c_str());
  }

  // frame widget to store image
  new puaImageFrame (
    1*DLG_DEF_SPACE,        y_in_win,
    1*DLG_DEF_SPACE + imgw, y_in_win + imgh,
    image
  );

  AboutDialog->close(); 
  AboutDialog->setSize(1 * imgw + 1 * txtw + 2 * DLG_DEF_SPACE,
                       1 * imgh + 1 * DLG_DEF_BUTTON_HEIGHT + 3 * DLG_DEF_SPACE);
  AboutDialog->centerOnScreen();
  AboutDialog->reveal();  
}

static void view_fullsc_cb(puObject *obj)
{
  int nFullscreen;
  int nX, nY;
  if (cfgfile->getInt("video.fullscreen.fUse"))
  {
    nFullscreen = 0;
    nX          = cfgfile->getInt("video.resolution.window.x", 800);
    nY          = cfgfile->getInt("video.resolution.window.y", 600);
  }
  else
  {
    nFullscreen = 1;
    nX          = cfgfile->getInt("video.resolution.fullscreen.x", 800);
    nY          = cfgfile->getInt("video.resolution.fullscreen.y", 600);
  }
  #ifdef linux
  // On Linux we can switch the mode on-the-fly.
  // This also puts the fullscreen flag back into the config file.
  Video::setupScreen(nX, nY, nFullscreen);
  #else
  // Other platforms need to put the value back into the
  // config file and restart manually.
  cfgfile->setAttributeOverwrite("video.fullscreen.fUse", nFullscreen);
  new CGUIMsgBox(_("Please save your configuration and restart CRRCSim!"));
  #endif
}

static void  view_wind_cb(puObject *obj)
{
  new CGUIViewWindDialog();
}

static void view_train_cb(puObject *obj)
{
  if (Global::training_mode)
    Global::training_mode = 0;
  else
    Global::training_mode = 1;
}

static void view_verbosity_cb(puObject *obj)
{
  if (Global::nVerbosity == 3)
    Global::nVerbosity = 0;
  else
    Global::nVerbosity++;
}

static void view_hudcompass_cb(puObject *obj)
{
  if (Global::HUDCompass)
    Global::HUDCompass = 0;
  else
    Global::HUDCompass = 1;
}

static void view_windvectors_cb(puObject *obj)
{
  if (Global::windVectors == 2)
    Global::windVectors = 0;
  else
    Global::windVectors++;
}

static void view_modelwindow_cb(puObject *obj)
{
  if (Global::modelView == 2)
    Global::modelView = 0;
  else
    Global::modelView++;
}

static void view_zoomin_cb(puObject *obj)
{
  zoom_in();
}

static void view_zoomout_cb(puObject *obj)
{
  zoom_out();
}

static void view_unzoom_cb(puObject *obj)
{
  zoom_reset();
}

static void quitDialogCallback(int choice)
{
  if (choice == CRRC_DIALOG_OK)
  {
    options_saveToFile();
  }
  Global::Simulation->quit();
}

static void view_testmode_cb(puObject *obj)
{
  Global::testmode = !Global::testmode;
  if (Global::testmode)
    activate_test_mode();
  else
    leave_test_mode();
}


/** \brief The GUI's "idle" function.
 *
 *  This function will be called from the main loop while
 *  the GUI is active. Its main purpose is to propagate
 *  the interface's input values to the CGUIMain object.
 *  A dialog that needs to evaluate the input signals may
 *  then access this local copy of the values by calling
 *  CGUIMain::getInputValues()
 */
void CGUIMain:: GUI_IdleFunction(TSimInputs *in)
{
  CRRCDialog *dlg = CRRCDialog::getToplevel();
  
  Global::gui->setInputValues(in);
  if (dlg != NULL)
  {
    dlg->update();
  }
}

void CGUIMain::setInputValues(TSimInputs *in)
{
  memcpy(&input, in, sizeof(TSimInputs));
}

TSimInputs* CGUIMain::getInputValues()
{
  return &input;
}

