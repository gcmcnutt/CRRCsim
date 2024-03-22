/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2004 - Joel Lienard, Lionel Cailler (original authors)
 *   Copyright (C) 2004, 2005, 2008 - Jens Wilhelm Wulf 
 *   Copyright (C) 2005-2009 - Jan Reucker
 *   Copyright (C) 2005 - Joel Lienard
 *   Copyright (C) 2008 - Olivier Bordes
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
 *  \file tx_audio.cpp
 *
 *  Definition of the AUDIO interface for CRRCsim.
 *
 *  This class will read signals from a PPM transmitter
 *  using a sound input device.
 */

#include "inputdev_audio.h"

#include <stdio.h>
#include <cstdlib>


static paAudioData  AudioData;


// --------------------------------------------------------------------------
/** This routine will be called by the PortAudio engine when audio is needed.
 *  It may be called at interrupt level on some machines so don't do anything
 *  that could mess up the system like calling malloc() or free().
 *  Purpose of this routine is to update a rotating buffer
 *  index of buffer is AudioData.frameIndex
 *  buffer size is AudioData.frameSize
 *  number of samples stored since audio_rc_open() called
 */
#if PORTAUDIO > 0
#if PORTAUDIO == 18
static int recordCallback( void *inputBuffer, void *outputBuffer,
                           unsigned long framesPerBuffer,
                           PaTimestamp outTime, void *userData )
#else
static int recordCallback( const void *inputBuffer, void *outputBuffer,
                           unsigned long framesPerBuffer,
                           const PaStreamCallbackTimeInfo *outTime,
                           PaStreamCallbackFlags b, void *userData )
#endif
{
  paAudioData *data = (paAudioData*)userData;
  SAMPLE *rptr = (SAMPLE*)inputBuffer;
  SAMPLE *wptr;
  unsigned long i;
  unsigned long framesLeft = data->frameSize - data->frameIndex;

  /* Prevent unused variable warnings. */
  outputBuffer = NULL;
  outTime = 0;

#if PORTAUDIO != 18
  b = 0;
#endif

  data->SamplesCounter += framesPerBuffer;

  wptr = &data->recordedSamples[data->frameIndex];
  if( framesPerBuffer <= framesLeft )
  {
    for( i=0; i<framesPerBuffer; i++ )
    {
      *wptr++ = *rptr++;
      #if NUM_CHANNELS == 2
      // skip second channel
      rptr++;
      #endif
    }
  }
  else
  {
    for( i=0; i<framesLeft; i++ )
    {
      *wptr++=*rptr++;
      #if NUM_CHANNELS == 2
      // skip second channel
      rptr++;
      #endif
    }

    wptr = &data->recordedSamples[0];
    for( i=0; i<framesPerBuffer-framesLeft; i++ )
    {
      *wptr++=*rptr++;
      #if NUM_CHANNELS == 2
      // skip second channel
      rptr++;
      #endif
    }
  }

  data->frameIndex=(data->frameIndex+framesPerBuffer)%data->frameSize;

  return 0;
}
#endif

// --------------------------------------------------------------------------

/**
 *  Create an audio interface.
 */
T_TX_InterfaceAudio::T_TX_InterfaceAudio()
  : cname("audio"), sDevName("")
{
#if DEBUG_TX_INTERFACE > 0
  printf("T_TX_InterfaceAudio::T_TX_InterfaceAudio()\n");
#endif
}


/**
 *  Delete an audio interface and close the input device.
 *
 */
T_TX_InterfaceAudio::~T_TX_InterfaceAudio()
{
#if DEBUG_TX_INTERFACE > 0
  printf("T_TX_InterfaceAudio::~T_TX_InterfaceAudio()\n");
#endif
  audio_rc_close();
  delete mixer;
}


/**
 *  Initialize an audio interface from the configuration file
 *  and try to open the input device.
 *
 *  \param config Pointer to the global configuration structure
 */
int T_TX_InterfaceAudio::init(SimpleXMLTransfer* config)
{
#if DEBUG_TX_INTERFACE > 0
  printf("int T_TX_InterfaceAudio::init(SimpleXMLTransfer* config)\n");
#endif
  T_TX_InterfacePPM::init(config);

  mixer = new T_TX_Mixer(this, config, "inputMethod.audio");
  
  SimpleXMLTransfer *tag = config->getChild("inputMethod.audio", true);
  sDevName = tag->attribute("device_name", "NOT_SPECIFIED");
  
  if (audio_rc_open() == 0)
    return(0);
  else
  {
    errMsg = "Error opening audio interface";
    return(-1);
  }
}


/**
 *  Get the current channel values from the interface.
 *
 *  \param inputs Pointer to a structure that stores the input values.
 */
void T_TX_InterfaceAudio::getInputData(TSimInputs* inputs)
{
#if DEBUG_TX_INTERFACE > 1
  printf("void T_TX_InterfaceAudio::getInputData(TSimInputs* inputs)\n");
#endif
  
  get_data_from_audio_interface(rc_channel_values);
  T_TX_InterfacePPM::getInputData(inputs);
}


/**
 *  Write all configuration data back into the config structure.
 *
 *  \param config Pointer to the global configuration structure.
 */
void T_TX_InterfaceAudio::putBackIntoCfg(SimpleXMLTransfer* config)
{
#if DEBUG_TX_INTERFACE > 0
  printf("int T_TX_InterfaceAudio::putBackIntoCfg(SimpleXMLTransfer* config)\n");
#endif

  SimpleXMLTransfer *tag = config->getChild("inputMethod.audio", true);
  tag->setAttributeOverwrite("device_name", sDevName);

  T_TX_InterfacePPM::putBackIntoCfg(config);
  mixer->putBackIntoCfg(config);
}


/**
 *  Close the audio input device.
 *
 *  \retval  0  success
 *  \retval -1  failure
 */
int T_TX_InterfaceAudio::audio_rc_close()
{
  int iRetCode = 0;

#if PORTAUDIO > 0
  PaError    err = paNoError;

  if (stream != NULL)
  {
    err = Pa_CloseStream( stream );
  }
  if( err == paNoError )
  {
    if (AudioData.recordedSamples != NULL)
    {
      free(AudioData.recordedSamples);
      free(AudioData.synchronizedSamples);
    }
  
    Pa_Terminate();
    
    printf("audio device closed\n");
  }
  else
  {
    Pa_Terminate();
    fprintf( stderr, "An error occured while using the portaudio stream in audio_rc_close\n" );
    fprintf( stderr, "Error number: %d\n", err );
    fprintf( stderr, "Error message: %s\n", Pa_GetErrorText( err ) );
  
    iRetCode = -1;
  }
#endif
  
  return iRetCode;
}


int T_TX_InterfaceAudio::get_data_from_audio_interface(float *values)
{
  static int nvals=0;

#if PORTAUDIO > 0
  static float vals[11];
  int i,n;
  
  /*
   * there is no need to update values more than 50 times a second
   * then lest check here every (SAMPLE_RATE/50) recorded samples
   */

  Pa_Sleep (1); // not sure this is usefull

  n=AudioData.SamplesCounter;
  if (n-AudioData.LastSamplesCounter>SAMPLE_RATE/50)
  {

    AudioData.LastSamplesCounter = n;                         // lets update values
    if(get_data(vals,&n))
    {
      for(i=0;i<11;i++)
        values[i]=vals[i];
      nvals = n;
    }
  }
  else
    /*
     * LastSamplesCounter & SamplesCounter saturation management
     */
    if (n<AudioData.LastSamplesCounter)
      AudioData.LastSamplesCounter = n;

#endif

  return nvals;
}


/**
 *  Open the audio input device.
 */
int T_TX_InterfaceAudio::audio_rc_open()
{
#if PORTAUDIO > 0
  int     i, numDevices;
  PaError err;
  const PaDeviceInfo *pdi;

  #if PORTAUDIO != 18
  PaStreamParameters inStreamParm;
  #endif

  err = Pa_Initialize();
  if( err != paNoError )
    goto error;

  /* number of all input AND output devices */
  #if PORTAUDIO == 18
  numDevices = Pa_CountDevices();
  #else
  numDevices = Pa_GetDeviceCount();
  #endif
  
  printf("Number of total audio input/output devices : %d.\n", numDevices );
  for( i = 0; i < numDevices; i++ )
  {
    pdi = Pa_GetDeviceInfo( i );
    printf("Device(%d) name  \t: %s.\n", i, pdi->name);
    printf("\tMax Inputs   = %d\n", pdi->maxInputChannels  );
    printf("\tMax Outputs  = %d\n", pdi->maxOutputChannels  );
  }

  inputDevice = -1;   // mark as invalid
  
  /* Find the correct input device.
   *
   * The complicated algorithm from earlier versions was discarded. Just look
   * for a device name that matches the configured device name and make sure
   * that it has enough input channels.
   */
  for( i = 0; i < numDevices; i++ )
  {
    pdi = Pa_GetDeviceInfo( i );
    
    if ((sDevName == pdi->name)
          &&
        (pdi->maxInputChannels >= NUM_CHANNELS))
    {
      /* found device from configuration file */
      printf("Audio device specified in configuration file (%s) found with ID %d\n", pdi->name, i);
      inputDevice = i;
      break;
    }
  }
  
  if (inputDevice < 0)
  {
    // no match found, use default input device
    #if PORTAUDIO == 18
    inputDevice = Pa_GetDefaultInputDeviceID();
    #else
    inputDevice = Pa_GetDefaultInputDevice();
    #endif
    fprintf(stderr, "Soundcard specified in configuration file (%s) not found,\n", sDevName.c_str());
    fprintf(stderr, "falling back to default (ID %d)\n", inputDevice);
  }
  
  /* now we should have a valid device ID, try to configure the device */
  if ((inputDevice >=0) && (inputDevice < numDevices))
  {
    pdi = Pa_GetDeviceInfo( inputDevice );
    printf("Using input device with ID %d (%s)\n", inputDevice, pdi->name );

    /*  check configuration */
    if (pdi->maxInputChannels < NUM_CHANNELS)
    {
      fprintf(stderr, 
              "no audio input device available, please configure your computer to support an input audio device.\n");
      Pa_Terminate();
      return (-1);
    }

    /*  configure device */

    AudioData.frameSize = NUM_SAMPLES; /* Record for a few samples. */
    AudioData.frameIndex = 0;
    AudioData.LastSamplesCounter = 0;
    AudioData.SamplesCounter = 0;
    AudioData.synchroIndex = 0;
    
    AudioData.recordedSamples = (SAMPLE *) malloc( AudioData.frameSize * sizeof(SAMPLE) );
    AudioData.synchronizedSamples = (SAMPLE *) malloc( AudioData.frameSize * sizeof(SAMPLE) );
    if (AudioData.recordedSamples == NULL || AudioData.synchronizedSamples == NULL)
    {
      printf("Could not allocate record array.\n");
      exit(1);
    }
    for( i=0; i<AudioData.frameSize; i++ )
      AudioData.recordedSamples[i] = AudioData.synchronizedSamples[i] = 0;

    #if PORTAUDIO != 18
    inStreamParm.device = inputDevice;
    inStreamParm.channelCount = NUM_CHANNELS;
    inStreamParm.hostApiSpecificStreamInfo = NULL;
    inStreamParm.sampleFormat = paFloat32;
    inStreamParm.suggestedLatency = pdi->defaultHighInputLatency;
    #endif

    /* Record some audio. -------------------------------------------- */

    #if PORTAUDIO == 18
    err = Pa_OpenStream(&stream,
                        inputDevice,
                        NUM_CHANNELS,
                        paFloat32,
                        NULL,
                        paNoDevice,
                        0,
                        paFloat32,
                        NULL,
                        SAMPLE_RATE,
                        1024,          /* frames per buffer */
                        0,             /* number of buffers, if zero then use default minimum */
                        0, //paDitherOff,    /* flags */
                        recordCallback,
                        &AudioData );
    #else
    err = Pa_OpenStream(&stream,
                        &inStreamParm,
                        NULL,
                        SAMPLE_RATE,
                        1024,
                        0,
                        recordCallback,
                        &AudioData);
    #endif

    if( err != paNoError )
      goto error;

    err = Pa_StartStream( stream );
    if( err != paNoError )
      goto error;

    // save the interface device data for the configuration
    sDevName    = pdi->name;
    
    return 0;
  }
  

error:
  Pa_Terminate();
  stream= 0;// for prevent more error on PAlib
  fprintf( stderr, "An error occured while using the portaudio stream in audio_rc_open\n" );
  fprintf( stderr, "Error number: %d\n", err );
  fprintf( stderr, "Error message: %s\n", Pa_GetErrorText( err ) );
  return -1;

#else  // PORTAUDIO == 0
  return -1;    // make it impossible to use this interface
#endif
}


/**
 * Process the recorded audio data and determine the channel values.
 *
 * \param values    Pointer to array that stores the channel values
 * \param nvalues   Number of channels detected will be stored here
 * \return          Always 1
 */
int T_TX_InterfaceAudio::get_data(float *values,int *nvalues)
{
  float x, px, max, min, moy;
  int nval = AudioData.frameSize;
  int i, j, chanel=0;
  float time, dt;
  float *sig = AudioData.recordedSamples;
  float *syn = AudioData.synchronizedSamples;

  max= -100000;
  min=  100000;
  
  for(i = 0; i < nval; i++)
  {
    x = sig[i];
    if (x > max)
      max = x;
    if (x < min)
      min = x;
  }
  moy = (max + min) / 2;
  x = 0;
  chanel = -1;
  time = 0;
  j = (AudioData.frameIndex + 1024) % AudioData.frameSize;
  for(i = 0; i < nval; i++)
  {
    time++;
    if (time > 100)
    {                   // Synchro detection
      if (chanel < 0)
        chanel = 0; //start
      else if (chanel > 0)
        break; //end
    }
    px = x;
    x = syn[j] = sig[j];
    if (++j == AudioData.frameSize)
    {
      j = 0;
    }
    if ((x > moy) && (px < moy))
    {
      dt = (moy - px) / (x - px);
      if (chanel >= 0)
      {
        if (chanel == 0 && i < (nval / 2))
        {  
          AudioData.synchroIndex = j;  //for OSCILLO  synchro
        }
        if (chanel > 0)
        {
          values[chanel-1] = (time + dt) / 44.1 - 1.5;
        }
        chanel ++;
        if (chanel > 10)
          break;
      }
      time = -dt;
    }
  }
  *nvalues = chanel - 1;

  return 1;
}


/**
 * \brief Re-open the audio interface
 *
 * Close the currently open audio-interface device, and open
 * the audio device specified in the internal variables
 */
void T_TX_InterfaceAudio::reopen()
{
  /// \todo Error handling?
  audio_rc_close();
  audio_rc_open();
}


/**
 * Return a single sample of the synchronised audio signal.
 *
 * This function provides an interface to the audio data for
 * external functions like the oscilloscope.
 *
 * \param  i  sample number
 * \return    sampled value at index i
 */
float get_audio_signal(int i)
{
  // read signal from the buffer syncronized by the
  // last call to get_data()
  float *sig = AudioData.synchronizedSamples;
  int j = AudioData.synchroIndex + i;
  if ( j >=  AudioData.frameSize)
  {
    j -=  AudioData.frameSize;
  }
  return sig[j];                     
}


/**
 * Create a list of possible device names
 *
 * This method adds all available audio input devices to Devices
 * and returns the number of devices.
 *
 * \param Devices   List of audio input devices. This list will be cleared
 *                  by this method before adding the currently available
 *                  device names.
 * \return number of devices (equal to Devices.size())
 */
int T_TX_InterfaceAudio::getDeviceList(std::vector<std::string>& Devices)
{
  Devices.erase(Devices.begin(), Devices.end());
  
  #if PORTAUDIO > 0
  // Just in case the user decides to hotplug something while the
  // dialog is open: store all currently found devices in this
  // array to make sure that a changed ID can be detected.

  // calls to PA_Initialize() and PA_Terminate() should be harmless if
  // the lib is already initialized.
  if (Pa_Initialize() == paNoError)
  {
    const PaDeviceInfo *pdi;
    int numDevices;
    #if PORTAUDIO == 18
    numDevices = Pa_CountDevices();
    #else
    numDevices = Pa_GetDeviceCount();
    #endif
    printf(    "Number of total audio input/output devices : %d.\n", numDevices );
    for( int i = 0; i < numDevices; i++ )
    {
      pdi = Pa_GetDeviceInfo( i );
      printf("Device(%d) name  \t: %s.\n", i, pdi->name);
      printf("\tMax Inputs   = %d\n", pdi->maxInputChannels  );
      printf("\tMax Outputs  = %d\n", pdi->maxOutputChannels  );
      
      if (pdi->maxInputChannels >= NUM_CHANNELS)
      {
        Devices.push_back(pdi->name);
      }
    }
    
    Pa_Terminate();
  }
  else
  {
    // Initialization of PortAudio failed
  }
  #endif
  
  return Devices.size();
}
