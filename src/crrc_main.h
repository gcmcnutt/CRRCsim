/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2000, 2001, 2004 Jan Edward Kansky (original author)
 * Copyright (C) 2004, 2005, 2006, 2008 Jan Reucker
 * Copyright (C) 2004, 2005, 2006, 2008 Jens Wilhelm Wulf
 * Copyright (C) 2004 Kees Lemmens
 * Copyright (C) 2004 Lionel Cailler
 * Copyright (C) 2005 Joel Lienard
 * Copyright (C) 2005 Olivier Bordes
 * Copyright (C) 2006 Todd Templeton
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
  

#ifndef CRRC_MAIN_H
#define CRRC_MAIN_H

#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <string.h>

#ifdef WIN32
#  include <windows.h>
// DISABLED FOR DEV-CPP #  include "dlportio.h"
# endif

#if defined(__APPLE__) || defined(MACOSX)
 #include <unistd.h>
 #include <sys/types.h>
 #include <sys/mman.h>
 #include <sys/stat.h>
 #include <sys/param.h> // for MAXPATHLEN
 #include <time.h>
 #include <sys/time.h>
#endif  // __APPLE__
#ifdef linux
 #include <sys/types.h>
 #include <sys/mman.h>
 #include <sys/stat.h>
 #include <time.h>
 #include <sys/time.h>
#endif

#include <fcntl.h>
#include <math.h>

#include "config.h"
#include "defines.h"
#include "crrc_soundserver.h"
#include "crrc_sound.h"
#include "mouse_kbd.h"
#include "CTime.h"
#include <crrc_config.h>
#include "mod_misc/SimpleXMLTransfer.h"
#include "mod_inputdev/inputdev.h"
#include "mod_fdm/fdm.h"




/*****************************************************************************/
// Functions define inside crrc_main.c and used in- or outside crrc_main.c :

void initialize_flight_model();
void set_aux(int aux_num, int setting);
void activate_test_mode();
void leave_test_mode();
std::string reconfigureInputMethod(bool boRevertToMouse = true);

/**
 * todo: maybe this should be moved to somewhere else...
 */
void Init_mod_windfield();

/**
 * Tries to load the airplane specified in the config file.
 * Throws an exception on error.
 */
void loadAirplane();

void write_globals_into_config();

/// Exit from CRRCsim as clean as possible
void crrc_exit(int exit_code, const char *errmsg = NULL);


/*****************************************************************************/
//Variables used outside crrc_main.c :

// Configuration :
extern T_Config*  cfg;

extern CRRCMath::Vector3 player_pos;

extern char scenery_filename[256];

// Interface to TX
extern T_TX_Interface* TXInterface;

extern CTime *crrc_time;

// Variometer sound
extern T_VariometerSound *vario_sound;
extern int vario_sound_channel;

#endif  // CRRC_MAIN_H
