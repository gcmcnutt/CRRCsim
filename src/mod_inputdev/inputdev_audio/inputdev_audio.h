/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2004 - Joel Lienard, Lionel Cailler (original authors)
 *   Copyright (C) 2004, 2005, 2008 - Jens Wilhelm Wulf
 *   Copyright (C) 2005-2009 - Jan Reucker
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
 *  \file inputdev_audio.h
 *
 *  Declaration of the AUDIO interface for CRRCsim.
 *
 *  This class will read signals from a PPM transmitter
 *  using a sound input device.
 */
#ifndef TX_INTERFACE_AUDIO_H
#define TX_INTERFACE_AUDIO_H

#include <string>

#include "../inputdev_PPM/inputdev_PPM.h"
#include "../../mod_misc/SimpleXMLTransfer.h"
#include <crrc_config.h>

#if PORTAUDIO > 0
#include <portaudio.h>
#endif

#define SAMPLE_RATE   44100
#define NUM_SAMPLES    3000
#define NUM_CHANNELS      1


typedef float SAMPLE;

typedef struct
{
  int     frameIndex;  /* Index into sample array. */
  int     frameSize;
  int     SamplesCounter;
  int     LastSamplesCounter;
  SAMPLE  *recordedSamples;
  int     synchroIndex;
  SAMPLE  *synchronizedSamples;
} paAudioData;


#if PORTAUDIO != 18
typedef void PortAudioStream;
#define PaStream PortAudioStream
#endif


float get_audio_signal(int i);


/** \brief The AUDIO interface
 *
 *  A PPM interface which acquires the Tx's signal through the
 *  sound card.
 */
class T_TX_InterfaceAudio : public T_TX_InterfacePPM
{
  public:
   T_TX_InterfaceAudio();
   virtual ~T_TX_InterfaceAudio();
   
   /**
    * Initialize interface. Read further config data from a file, if necessary.
    */
   int init(SimpleXMLTransfer* config);
   
   /**
    * Set current input data. If some value is not available, the value 
    * is not overwritten.
    */
   void getInputData(TSimInputs* inputs);
   
   /**
    * Write configuration back
    */
   virtual void putBackIntoCfg(SimpleXMLTransfer* config);

   /**
    * Get input method
    */
   int inputMethod() { return(T_TX_Interface::eIM_audio); };
   
   /** \brief Get the name of the interface for the config file
    *
    *  Returns the name of the interface. It should be a single
    *  word, only lower-case characters.
    */
   const char * getConfigName() {return cname;};
   
   /**
    * Re-open the audio device
    */
   void reopen();
  
  /**
   * Create a list of possible audio devices
   */
  static int getDeviceList(std::vector<std::string>& Devices);
   
  private:
    const char *cname;        ///< name in the config file
  
    // The device is identified by its device name
    std::string sDevName;     ///< name of the device as reported by Portaudio
    int         inputDevice;  ///< internal device index for Portaudio

    #if PORTAUDIO > 0
    PortAudioStream   *stream;
    #endif

    int   audio_rc_open();
    int   audio_rc_close();
    int   get_data_from_audio_interface(float *values);
    int   get_data(float *values, int *nvalues);
};

#endif

