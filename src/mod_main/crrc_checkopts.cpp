/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2004 Kees Lemmens (original author)
 * Copyright (C) 2004, 2005, 2008 Jens Wilhelm Wulf
 * Copyright (C) 2005, 2006, 2007, 2008, 2009 Jan Reucker
 * Copyright (C) 2005 Lionel Cailler
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


#include <unistd.h>

#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>

#include <crrc_config.h>

extern char   *optarg;
extern int    optind;

#include "../global.h"
#include "../crrc_main.h"

// Prototypes for local functions
static void crrc_version_info();
static void crrc_usage(char *progname);

#define OPTION_STRING "b:c:d:fg:hi:j:l:m:s:u:vVw:x:y:"

/**
 * Print usage information and exit
 *
 * \param progname The name as which the program was called.
 */
static void crrc_usage(char *progname)
{
  fprintf(stderr,"\nUsage  : %s [options] [<plane>]\n",progname);
  fprintf(stderr,"\nNote that these options override default settings in crrcsim.xml !\n");
  fprintf(stderr,  "Options:\n");
  fprintf(stderr,  "         -h             : display this message\n");
  fprintf(stderr,  "         -l <string>    : location/scenery file with path (e.g. scenery/davis-orig.xml)\n");
  fprintf(stderr,  "                          Use this option before others, otherwise they might be overwritten.\n");
  fprintf(stderr,  "         -b <nr:string> : joystick buttonnr function: RESUME|RESET|PAUSE|ZOOMIN|ZOOMOUT|INCTHROTTLE|DECTHROTTLE\n");
  fprintf(stderr,  "         -c <value>     : color_depth in bits per pixel\n");
  fprintf(stderr,  "         -d <value>     : wind direction in deg (0-360)\n");
  fprintf(stderr,  "         -f             : use fullscreen\n");
  fprintf(stderr,  "         -g <string>    : specify config file\n");
  fprintf(stderr,  "         -i <string>    : input method : KEYBOARD|MOUSE|JOYSTICK|RCTRAN|SERIAL2|PARALLEL|AUDIO|MNAV|ZHENHUA\n");
  fprintf(stderr,  "         -m <string>    : mouse x motion : AILERON|RUDDER\n");
  fprintf(stderr,  "         -s <on/off>    : sound on/off\n");
  fprintf(stderr,  "         -u <on/off>    : user interface on/off\n");
  fprintf(stderr,  "         -w <value>     : wind velocity in ft/sec\n");
  fprintf(stderr,  "         -x <value>     : x_resolution in pixels\n");
  fprintf(stderr,  "         -y <value>     : y_resolution in pixels\n");
  fprintf(stderr,  "         -v             : Show input values\n");
  fprintf(stderr,  "         -v             : Show current field of view\n");
  fprintf(stderr,  "         -v             : Show frames per second\n");
  fprintf(stderr,  "         -V             : print version info and exit\n");
  fprintf(stderr, "\n");
}

/**
 * returns 
 *   0  everything OK, let's start
 *  -1  exit with error
 */
int crrc_checkopts (int                argc,
                    char*              argv[], 
                    SimpleXMLTransfer* cfgfile,
                    T_Config*          cfg)  /* check and set options flags */
{
  int c;
  int buttonnr;
  int opt_err = 0;
  int new_res_x = 0;
  int new_res_y = 0;

  while ((c = getopt(argc, argv, OPTION_STRING)) != -1)
  {
    switch (c)
    {
      case 'b':
        if (! isdigit(optarg[0]) || optarg[1] != ':')
        {
          opt_err = 1;
          break;
        }
        buttonnr=atoi(optarg);
        if(buttonnr > MAXJOYBUTTON)
        {
          fprintf(stderr, "illegal button nr: %d\n", buttonnr);
          crrc_exit(CRRC_EXIT_FAILURE,"");
        }
        optarg += 2;
        if (strcasecmp(optarg,"RESUME")==0)
          Global::inputDev->joystick_bind_b[buttonnr] = TInputDev::RESUME;
        else if (strcasecmp(optarg,"RESET")==0)
          Global::inputDev->joystick_bind_b[buttonnr] = TInputDev::RESET;
        else if (strcasecmp(optarg,"PAUSE")==0)
          Global::inputDev->joystick_bind_b[buttonnr] = TInputDev::PAUSE;
        else if (strcasecmp(optarg,"DECTHROTTLE")==0)
          Global::inputDev->joystick_bind_b[buttonnr] = TInputDev::DECTHROTTLE;
        else if (strcasecmp(optarg,"INCTHROTTLE")==0)
          Global::inputDev->joystick_bind_b[buttonnr] = TInputDev::INCTHROTTLE;
        else if (strcasecmp(optarg,"ZOOMIN")==0)
          Global::inputDev->joystick_bind_b[buttonnr] = TInputDev::ZOOMIN;
        else if (strcasecmp(optarg,"ZOOMOUT")==0)
          Global::inputDev->joystick_bind_b[buttonnr] = TInputDev::ZOOMOUT;
        break;
      case 'c':
        cfgfile->setAttributeOverwrite("video.color_depth", optarg);
        break;
      case 'd':
        cfg->wind->setDirection((float)atof(optarg), cfg);
        break;
      case 'f':
        cfgfile->setAttributeOverwrite("video.fullscreen.fUse", "1");
        break;
      case 'g':
        // handled in crrc_main.cpp before this is called
        break;
      case 'h':
        crrc_usage(argv[0]);
        crrc_exit(CRRC_EXIT_SUCCESS,"");
        break;
      case 'i':
        cfgfile->setAttributeOverwrite("inputMethod.method", optarg);
        break;
      case 'l': /* airport location */
        cfg->setLocation(optarg, cfgfile);
        break;
      case 'm':
        if (strcasecmp(optarg,"AILERON")==0)
          Global::inputDev->mouse_bind_x = T_AxisMapper::AILERON;
        else if (strcasecmp(optarg,"RUDDER")==0)
          Global::inputDev->mouse_bind_x = T_AxisMapper::RUDDER;
        break;
      case 's':
        if      (strcasecmp(optarg,"ON")==0)
          cfgfile->setAttributeOverwrite("sound.enabled", "1");
        if      (strcasecmp(optarg,"OFF")==0)
          cfgfile->setAttributeOverwrite("sound.enabled", "0");
        break;
      case 'u':
        if      (strcasecmp(optarg,"ON")==0)
          cfgfile->setAttributeOverwrite("video.enabled", "1");
        if      (strcasecmp(optarg,"OFF")==0)
          cfgfile->setAttributeOverwrite("video.enabled", "0");
        break;
      case 'v':
        Global::nVerbosity++;
        break;
      case 'w':
        cfg->wind->setVelocity((float)atof(optarg));
        break;
      case 'x':
        new_res_x = atoi(optarg);
        break;
      case 'y':
        new_res_y = atoi(optarg);
        break;
      case 'V':
        // should already be evaluated
        break;
      default:
        opt_err = 1;
    }
    
    if (opt_err)
    {
      crrc_usage(argv[0]);
      return(-1);
    }
  }
   
  // x and y resolution must be stored after all other flags have
  // been evaluated to make sure that we really know if the user
  // wants fullscreen or windowed mode.
  int use_fullscreen = cfgfile->getInt("video.fullscreen.fUse", 0);
  if (new_res_x > 0)
  {
    if (use_fullscreen)
    {
      cfgfile->setAttributeOverwrite("video.resolution.fullscreen.x", new_res_x);
    }
    else
    {
      cfgfile->setAttributeOverwrite("video.resolution.window.x", new_res_x);
    }
  }
  if (new_res_y > 0)
  {
    if (use_fullscreen)
    {
      cfgfile->setAttributeOverwrite("video.resolution.fullscreen.y", new_res_y);
    }
    else
    {
      cfgfile->setAttributeOverwrite("video.resolution.window.y", new_res_y);
    }
  }
   
  if (argc - optind > 0)
  {
    cfgfile->setAttributeOverwrite("airplane.file", argv[optind]);
  }
  
  return 0;
}


/**
 * This function checks if the "print version" option
 * was given on the command line.
 * \param argc  Number of arguments as passed to main()
 * \param argv  List of arguments as passed to main()
 *
 * \retval true   -V option was found and version info was printed
 * \retval false  -V option was not found
 */
bool crrc_checkversionopt(int argc, char* argv[])
{
  int c;
  bool boFoundV = false;

  while ((c = getopt(argc, argv, OPTION_STRING)) != -1)
  {
    if (c == 'V')
    {
      crrc_version_info();
      boFoundV = true;
    }
  }
  
  // reset option index for the second parser run
  optind = 1;

  return boFoundV;
}


/**
 * Print version and configuration information
 */
static void crrc_version_info()
{
  std::cout << "CRRCsim ";
  
  if (std::string(PACKAGE_VERSION).compare("dev") == 0)
  {
    std::cout << "(development build, " << __DATE__ << ")";
  }
  else
  {
    std::cout << PACKAGE_VERSION;
  }
  std::cout << std::endl << std::endl;
    
  std::cout << "Configuration options:" << std::endl;
  
  // Portaudio library options
  #if PORTAUDIO > 0
  std::cout << "  audo interface supported (Portaudio V" << PORTAUDIO << ")";
  #else
  std::cout << "  audio interface not support";
  #endif
  std::cout << std::endl;
  
  // Mouse wheel support
  #ifdef SDL_WITHOUT_MOUSEWHEEL
  std::cout << "  mouse wheel not supported";
  #else
  std::cout << "  mouse wheel supported";
  #endif
  std::cout << std::endl;
  
  // Wind data import support
  #if WINDDATA3D > 0
  std::cout << "  wind data import supported";
  #else
  std::cout << "  wind data import not supported";
  #endif
  std::cout << std::endl;
}
