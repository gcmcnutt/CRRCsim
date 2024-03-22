/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2004 Kees Lemmens (original author)
 * Copyright (C) 2005, 2006, 2007, 2008, 2009 Jens Wilhelm Wulf
 * Copyright (C) 2005, 2006, 2007, 2008, 2010 Jan Reucker
 * Copyright (C) 2005 Joel Lienard
 * Copyright (C) 2006 Todd Templeton
 * Copyright (C) 2008 Olivier Bordes
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
  

#include "global.h"
#include "i18n.h"
#include "aircraft.h"
#include "SimStateHandler.h"
#include "crrc_main.h"
#include "defines.h"
#include "mod_windfield_config.h"
#include "zoom.h"
#include "GUI/crrc_gui_main.h"
#include "GUI/crrc_dialog.h"
#include "mod_misc/lib_conversions.h"

#if DEBUG_THERMAL_SCRSHOT == 1
# include "mod_windfield/windfield.h"
# include "mod_fdm/fdm.h"
#endif

/*****************************************************************************/

void key_down(SDL_keysym *keysym)
{
  static int current_aux = -1;
  int received_begin_aux = 0;

  switch (keysym->unicode)
  {
    case 'q':
    case 'Q':
      if (Global::gui) 
        Global::gui->doQuitDialog();
      else
        Global::Simulation->quit();
      break;
      
    case '<':
      {
        Global::slowTimeScale += 0.5;
        if (Global::slowTimeScale > 3.)
          Global::slowTimeScale = 3.;
        std::string txt = _("Slow motion time dilatation = ");
        txt += ftoStr(Global::slowTimeScale*100,3,0,TRUE,FALSE);
        txt += "%";
        LOG(txt);
      }
      break;
      
    case '>':
      {
        Global::slowTimeScale -= 0.5;
        if (Global::slowTimeScale < 1.5)
          Global::slowTimeScale = 1.5;
        std::string txt = _("Slow motion time dilatation = ");
        txt += ftoStr(Global::slowTimeScale*100,3,0,TRUE,FALSE);
        txt += "%";
        LOG(txt);
      }
      break;
      
    default:
      switch (keysym->sym)
      {
        case SDLK_SPACE:
          if (Global::slowMotion == FALSE)
          {
            Global::slowMotion = TRUE;
            std::string txt = _("Slow motion is ON, time dilatation = ");
            txt += ftoStr(Global::slowTimeScale*100,3,0,TRUE,FALSE);
            txt += "%";
            LOG(txt);
          }
          else
          {
            Global::slowMotion = FALSE;
            LOG(_("Slow motion is OFF."));
          }
          break;
          
        case SDLK_r:
          if (! CRRCDialog::getToplevel() )
          {
            if (Global::gui && Global::gui->isVisible())
            {
              Global::gui->hide();
            }
            Global::Simulation->reset();
          }
          break;
          
        case SDLK_p:
          if (!(Global::gui && Global::gui->isVisible()))
          {
            if (Global::Simulation->getState() == STATE_PAUSED)
            {
              Global::Simulation->resume();
            }
            else
            {
              Global::Simulation->pause();
              LOG(_("Press <p> again to resume."));
            }
          }
          break;
          
        case SDLK_d:
          if (!(Global::gui && Global::gui->isVisible()))
          {
            if (Global::testmode)
            {
              Global::Simulation->reset();
            }
            else
            {
              activate_test_mode();
            }
          }
          break;
          
        case SDLK_g:
          Global::TXInterface->toggleRetract();
          break;
        
        case SDLK_b:
          Global::TXInterface->toggleSpoiler();
          break;
        
        case SDLK_t:
          if (Global::training_mode == FALSE)
          {
            Global::training_mode = TRUE;
            LOG(_("Training mode is ON, thermals are visible."));
          }
          else
          {
            Global::training_mode = FALSE;
            LOG(_("Training mode is OFF."));
          }
          break;
          
        case SDLK_v:
          if (Global::nVerbosity == 3)
          {
            Global::nVerbosity = 0;
          }
          else
          {
            Global::nVerbosity++;
          }
          break;
          
        case SDLK_h:
          if (Global::HUDCompass)
          {
            Global::HUDCompass = 0;
          }
          else
          {
            Global::HUDCompass = 1;
          }
          break;
          
        case SDLK_w:
          if (Global::windVectors == 2)
          {
            Global::windVectors = 0;
            LOG(_("Wind vectors are OFF."));
          }
          else
          {
            Global::windVectors++;
            if (Global::windVectors == 1)
              LOG(_("Wind vectors are ON. Color by total velocity."));
            else
              LOG(_("Wind vectors are ON. Color by vertical velocity."));
          }
          break;
          
        case SDLK_m:
          if (Global::modelView == 2)
          {
            Global::modelView = 0;
            LOG(_("Model view window is OFF."));
          }
          else
          {
            Global::modelView++;
            if (Global::modelView == 1)
              LOG(_("Model view window is ON."));
            else
              LOG(_("Model view window is AUTO."));
          }
          break;
          
        case KEY_ZOOM_IN:
          if (Global::inputDev->zoom_control == TInputDev::KEYBOARD)
          {
            zoom_in();
          }
          break;
          
        case KEY_ZOOM_OUT:
          if (Global::inputDev->zoom_control == TInputDev::KEYBOARD)
          {
            zoom_out();
          }
          break;
          
        case SDLK_KP5:
          Global::TXInterface->centerControls();
          break;
          
        case SDLK_LEFT:
        case SDLK_KP4:
          Global::TXInterface->move_rudder(0.05);
          break;
          
        case SDLK_RIGHT:
        case SDLK_KP6:
          Global::TXInterface->move_rudder(-0.05);
          break;
          
        case SDLK_KP7:
          Global::TXInterface->move_aileron(-0.05);
          break;
          
        case SDLK_KP9:
          Global::TXInterface->move_aileron(0.05);
          break;
          
        case SDLK_UP:
        case SDLK_KP8:
          Global::TXInterface->move_elevator(0.05);
          break;
          
        case SDLK_DOWN:
        case SDLK_KP2:
          Global::TXInterface->move_elevator(-0.05);
          break;
          
        case KEY_THROTTLE_MORE:
          Global::TXInterface->increase_throttle();
          break;
          
        case KEY_THROTTLE_LESS:
          Global::TXInterface->decrease_throttle();
          break;
        
        case SDLK_F1:
          current_aux = 1;
          received_begin_aux = 1;
          break;

        case SDLK_F2:
          current_aux = 2;
          received_begin_aux = 1;
          break;

        case SDLK_F3:
          current_aux = 3;
          received_begin_aux = 1;
          break;

        case SDLK_F4:
          current_aux = 4;
          received_begin_aux = 1;
          break;

        case SDLK_1:
          if (current_aux >= 0)
            set_aux(current_aux, 1);
          break;

        case SDLK_2:
          if (current_aux >= 0)
            set_aux(current_aux, 2);
          break;

        case SDLK_3:
          if (current_aux >= 0)
            set_aux(current_aux, 3);
          break;
          
#if DEBUG_THERMAL_SCRSHOT == 1
        case SDLK_s:
          windfield_thermalScreenshot(Global::aircraft->getPos());
          break;
#endif

        case SDLK_c:
          switch (Global::aircraft->ReloadParams())
          {
            case 0:
              // nothing happened
              break;
              
            case 1:
              LOG(_("Reloading parameters"));
              break;
              
            case -1:
              LOG(_("Reloading parameters: error"));
              break;
          }
          break;
          
        default:
          Global::inputs.AddKey(keysym->unicode);
          break;
      }
      
      break;
  } 

  if (!received_begin_aux)
    current_aux = -1;
}

