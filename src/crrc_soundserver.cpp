/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2006, 2008, 2009 Jan Reucker (original author)
 * Copyright (C) 2005, 2008 Jens Wilhelm Wulf
 * Copyright (C) 2006 Todd Templeton
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
  

/**
 *  \file crrc_soundserver.cpp
 *
 *  The CRRCsim sound server and related functions.
 *
 *  Author: Jan Reucker
 *  eMail:  slowhand_47@gmx.de
 */

#include "crrc_soundserver.h"
#include "crrc_main.h"
#include "mod_misc/lib_conversions.h"


// --- generic functions ----------------------------------

/** \brief Print SDL_AudioSpec data
 *
 *  Prints the data contained in an SDL_AudioSpec in
 *  a user-friendly format.
 */
void PrintFormat(const char *title, SDL_AudioSpec *fmt)
{
  printf("%s: %d bit %s audio (%s) at %u Hz\n", 
              title, 
              (fmt->format&0xFF),
              (fmt->format&0x8000) ? "signed" : "unsigned",
              (fmt->channels > 2) ? "surround" :
              (fmt->channels > 1) ? "stereo" : "mono",
              fmt->freq);
}


/** \brief The sound callback
 *
 *  This callback routine is the low-level workhorse which
 *  interfaces the sound server to SDL. It is called by the
 *  SDL routines whenever the sound card accepts new input
 *  data.
 */
void snd_callback(void *_unused, Uint8 *stream, int len)
{
  Uint32 samples;
  CRRCAudioServer *server = CRRCAudioServer::getRunningInstance();
  
  if (!server->is_paused)
  {
    for (int i = 0; i < CRRC_AUDIO_CHANNELS; i++)
    {
      if (server->channel[i] != NULL && server->channel[i]->sample != NULL)
      {
        samples = len;
        Uint8  *pos = server->channel[i]->sample->getMixableData(server->channel[i]->playpos, &samples);
        
        // end of sample reached?
        if (samples == 0)
        {
          server->stopChannel(i);
        }
        else
        {
          SDL_MixAudio(stream, pos, samples, server->channel[i]->volume);
          server->channel[i]->playpos += samples;
        }
      }
    }
  }
}


// --- Implementation of class CRRCAudioServer ------------

CRRCAudioServer* CRRCAudioServer::instance = NULL;

/** \brief Initialize the audio server.
 *
 *  \param config Pointer to the XML config file.
 */
CRRCAudioServer::CRRCAudioServer(SimpleXMLTransfer *config)
  : audio_spec(NULL), is_paused(true)
{
  // Prepare config files
  config->makeSureAttributeExists("sound.samplerate", "48000");
  
  SDL_AudioSpec *desired;
  desired  = (SDL_AudioSpec*)malloc(sizeof(SDL_AudioSpec));
  audio_spec = (SDL_AudioSpec*)malloc(sizeof(SDL_AudioSpec));
  
  desired->freq = config->getInt("sound.samplerate");
  desired->format = AUDIO_S16SYS;
  desired->channels = 1;
  desired->callback = snd_callback;
  desired->userdata = NULL;

  desired->samples = 2048;
  if (desired->freq > 30000)
  {
    desired->samples *= 2;
  }
  
  for (int i = 0; i < CRRC_AUDIO_CHANNELS; i++)
  {
    channel[i] = NULL;
  }

  // try to open
  if ( SDL_OpenAudio(desired, audio_spec) < 0 )
  {
    std::string s = "Couldn't open audiodevice: ";
    s += SDL_GetError();
    fprintf(stderr, "%s\n", s.c_str());
    instance = NULL;
    throw std::runtime_error(s);
  }
  else
  {
    PrintFormat("Opened audio device", audio_spec);
    instance = this;
  }
  free(desired);
  
  double dModelVolume = config->getDouble("sound.model.vol", 1.0);
  if (dModelVolume > 1.0)
  {
    dModelVolume = 1.0;
    config->setAttributeOverwrite("sound.model.vol", "1.0");
  }
  ucModelVolume = dModelVolume * SDL_MIX_MAXVOLUME;

}



/** \brief Delete the sound server.
 *
 *  Stops the sound server and deletes all related data.
 */
CRRCAudioServer::~CRRCAudioServer()
{
  SDL_PauseAudio(1);

  // free any allocated samples
  for (int i = 0; i < CRRC_AUDIO_CHANNELS; i++)
  {
    if (channel[i] != NULL)
    {
      if (channel[i]->discard)
      {
        #if DEBUG_SOUND_SERVER > 0
        printf("Discarding sample %s.\n", channel[i]->sample->getName().c_str());
        #endif
        delete channel[i]->sample;
      }
      delete channel[i];
      channel[i] = NULL;
    }
  }
  free(audio_spec);
  SDL_CloseAudio();
}


/** \brief Store configurable values in config file
 *
 *  This method stores all configuration values in the
 *  given config file.
 *
 *  \param config Pointer to the XML config file.
 */
void CRRCAudioServer::putBackIntoConfig(SimpleXMLTransfer *config) const
{
  if (config != NULL)
  {
    float flModelVol = (float)ucModelVolume / (float)SDL_MIX_MAXVOLUME;
    std::string s = ftoStr(flModelVol, 1, 3);
    std::cout << "CRRCAudioServer::putBackIntoConfig: sound.model.vol == " << s << std::endl;
    config->setAttributeOverwrite("sound.model.vol", s);
  }
}


/** \brief Play a preloaded sample.
 *
 *  Adds the sample to the CRRCAudioServer. The sample will
 *  be played at the given volume, or at the maximum volume
 *  if this parameter is omitted.
 *  If all channels of the server are busy, the method will
 *  return -1, else it will return the number of the channel
 *  which now plays the sample.
 *
 *  \param sample Pointer to the sample to be played.
 *  \param volume playback volume
 *  \return channel number or -1 on error
 */
int CRRCAudioServer::playSample(T_SoundSample *sample, unsigned int volume)
{
  return addSample(sample, volume, false);
}


/** \brief Play a sample directly from a file.
 *
 *  Creates a temporary T_SoundSample object from a file,
 *  adds the sample to the CRRCAudioServer and discards
 *  the temporary T_SoundSample afterwards. The sample will
 *  be played at the given volume, or at the maximum volume
 *  if this parameter is omitted.
 *  If all channels of the server are busy, the method will
 *  return -1, else it will return the number of the channel
 *  which now plays the sample.
 *
 *  Compared to playing a preloaded sample, this method may
 *  introduce extra system load and a little sound delay,
 *  because the sample might have to be converted to the
 *  sound server's format.
 *
 *  \param filename Name of the .wav file to be played.
 *  \param volume playback volume
 *  \return channel number or -1 on error
 */
int CRRCAudioServer::playSample(const char *filename, unsigned int volume)
{
  int chan;
  T_SoundSample *sample = NULL;
  
  try
  {
    sample = new T_SoundSample(filename, audio_spec);
  }
  catch (std::runtime_error& e)
  {
    fprintf(stderr, "%s\n", e.what());
    return -1;
  }

  chan = addSample(sample, volume, true);
  if (chan < 0)
  {
    delete sample;
  }
  return chan;
}


/** \brief Add a sample to the server.
 *
 *  This handles the internal adding process and is called
 *  from all flavours of playSample(). It returns the
 *  number of the channel to which the sample was assigned
 *  or -1 if there was no more free channel.
 *  \param sample Pointer to the sample to be played.
 *  \param volume playback volume
 *  \param disc   Discard sample after playback?
 *  \return channel number or -1 on error
 */
int CRRCAudioServer::addSample(T_SoundSample *sample,
                                unsigned int volume,
                                bool disc)
{
  int ret = -1;

  if ((sample->getFrequency() != audio_spec->freq)
        ||
      (sample->getFormat() != audio_spec->format)
        ||
      (sample->getNumChannels() != audio_spec->channels))
  {
    // sample does not match playback format, convert it
    sample->convert(audio_spec);
  }
  
  SDL_LockAudio();
  for (int i = 0; i < CRRC_AUDIO_CHANNELS; i++)
  {
    if (channel[i] == NULL)
    {
      T_PlaybackContainer *pb = new T_PlaybackContainer;
      pb->sample = sample;
      pb->volume = volume;
      pb->discard = disc;
      pb->playpos = 0;
      channel[i] = pb;
      ret = i;
      #if DEBUG_SOUND_SERVER > 0
      printf("Added sample %s to channel %d.\n", sample->getName().c_str(), i);
      #endif
      break;
    }
  }
  SDL_UnlockAudio();
  
  #if DEBUG_SOUND_SERVER > 0
  if (ret < 0)
  {
    fprintf(stderr, "*** Unable to play sample %s: No free channels!\n", sample->getName().c_str());
  }
  #endif
  return ret;
}


/** \brief Get the server's audio format.
 *
 *  This method returns a pointer to the server's SDL_AudioSpec.
 */
SDL_AudioSpec* CRRCAudioServer::getAudioSpec() const
{
  return audio_spec;
}


/** \brief Stop playback on a channel.
 *
 *  This stops the sample playing on channel c. If the
 *  sample was created by the server, it will automatically
 *  be deleted.
 *
 *  \param c channel number
 */
void CRRCAudioServer::stopChannel(int c)
{
  if ((c >= 0) && (c < CRRC_AUDIO_CHANNELS))
  {
    if (channel[c] != NULL)
    {
      #if DEBUG_SOUND_SERVER > 0
      printf("Stopping sample %s on channel %d.\n", channel[c]->sample->getName().c_str(), c);
      #endif
      SDL_LockAudio();
      if (channel[c]->discard)
      {
        #if DEBUG_SOUND_SERVER > 0
        printf("Discarding sample %s.\n", channel[c]->sample->getName().c_str());
        #endif
        delete channel[c]->sample;
        
      }
      delete channel[c];
      channel[c] = NULL;
      SDL_UnlockAudio();
    }
  }
}


/** \brief Stop playback an all channels.
 *
 *  This stops all samples currently playing. All
 *  channels will be freed. All samples created by
 *  the server will be deleted.
 */
void CRRCAudioServer::stopAllChannels()
{
  for (int c = 0; c < CRRC_AUDIO_CHANNELS; c++)
  {
    stopChannel(c);
  }
}


/** \brief Set the volume of a channel.
 *
 *  Set a channel's playback volume to the given level. The
 *  vol parameter will be clamped to SDL_MIX_MAXVOLUME, which
 *  is usually defined as 128.
 *
 *  \param c channel number
 *  \param vol desired volume (0 ... SDL_MIX_MAXVOLUME)
 */
void CRRCAudioServer::setChannelVolume(int c, unsigned char vol)
{
  if ((c >= 0) && (c < CRRC_AUDIO_CHANNELS))
  {
    if (channel[c] != NULL)
    {
      if (vol > SDL_MIX_MAXVOLUME)
      {
        vol = SDL_MIX_MAXVOLUME;
      }
      channel[c]->volume = vol;
    }
  }
}


/**
 *  Set the volume for all models.
 *
 *  \param vol desired volume (0...SDL_MIX_MAXVOLUME)
 */
void CRRCAudioServer::setModelVolume(unsigned char vol)
{
  if (vol > SDL_MIX_MAXVOLUME)
  {
    vol = SDL_MIX_MAXVOLUME;
  }
  SDL_LockAudio();
  ucModelVolume = vol;
  SDL_UnlockAudio();
}


// --- Implementation of class T_SoundSample --------------

/** \brief Create an empty sound sample.
 *
 *  This ctor is mainly useful for derived classes which
 *  provide their own mechanism to create or read sound
 *  data.
 */
T_SoundSample::T_SoundSample(SDL_AudioSpec *fmt)
  : samplename(""), length(0), buffer(NULL)
{
  spec.format = fmt->format;
  spec.freq   = fmt->freq;
  spec.channels = fmt->channels;
}


/** \brief Create a sound sample from a file.
 *
 * Creates a sound sample with the desired format from
 * a file. A std::runtime_error will be thrown on error.
 * This includes:
 * - file does not exist
 * - file format not recognized or invalid
 * - conversion into desired format failed
 *
 * The conversion mechanism was shamelessly stolen from
 * the SDL_mixer library. Many thanks to the original
 * author(s).
 *
 * \param filename the file to be loaded
 * \param fmt desired audio format
 */
T_SoundSample::T_SoundSample(const char *filename, SDL_AudioSpec *fmt)
  : samplename(""), length(0), buffer(NULL)
{
  SDL_AudioSpec *ret = SDL_LoadWAV(filename, &spec, &buffer, &length);
  if (NULL == ret)
  {
    std::string s = "T_SoundSample: ";
    s += SDL_GetError();
    throw std::runtime_error(s);
  }
  #if DEBUG_SOUND_SERVER > 0
  else
  {
    PrintFormat(filename, &spec);
    printf("    (%u samples)\n", length);
  }
  #endif

  samplename    = filename;

  convert(fmt);
}


/** \brief Convert the sample to the given format.
 *
 *  Converts the sample to the audio format specified
 *  by fmt. If the conversion fails, a std::runtime_error
 *  exception might be thrown.
 *
 *  \param fmt The desired sample format.
 */
void T_SoundSample::convert(SDL_AudioSpec *fmt)
{
  SDL_AudioCVT  wavecvt;
  
  /* Build the audio converter and create conversion buffers */
  if (SDL_BuildAudioCVT(&wavecvt,
                        spec.format, 
                        spec.channels,
                        spec.freq,
                        fmt->format,
                        fmt->channels,
                        fmt->freq) < 0 )
  {
    SDL_FreeWAV(buffer);
    std::string s = "Could not initialize converter: ";
    s += SDL_GetError();
    throw std::runtime_error(s);
  }
  wavecvt.len = length & ~(getSampleSize()-1);
  wavecvt.buf = (Uint8 *)malloc(wavecvt.len * wavecvt.len_mult);
  if ( wavecvt.buf == NULL )
  {
    SDL_FreeWAV(buffer);
    std::string s = "Could not initialize converter: ";
    s += SDL_GetError();
    throw std::runtime_error(s);
  }
  memcpy(wavecvt.buf, buffer, length);
  SDL_FreeWAV(buffer);

  /* Run the audio converter */
  if (SDL_ConvertAudio(&wavecvt) < 0)
  {
    free(wavecvt.buf);
    std::string s = "Conversion failed: ";
    s += SDL_GetError();
    throw std::runtime_error(s);
  }
  
  /* conversion succeeded, fetch the results */
  buffer = wavecvt.buf;
  length = wavecvt.len_cvt;
  spec.format   = fmt->format;
  spec.channels = fmt->channels;
  spec.freq     = fmt->freq;

  #if DEBUG_SOUND_SERVER > 0
  printf("Converted ");
  PrintFormat(samplename.c_str(), &spec);
  printf("    (%u samples)\n", length);
  #endif
}


/** \brief Destroy the sound sample.
 *
 * Deletes the sound sample object.
 */
T_SoundSample::~T_SoundSample()
{
  SDL_FreeWAV(buffer);
}


/** \brief Get the length of the T_SoundSample
 *
 *  \return length of the sample
 */
Uint32 T_SoundSample::getLength() const
{
  return length;
}


/** \brief Get a pointer to a chunk of data.
 *
 *  This method returns a pointer to the sample buffer,
 *  pointing playCount samples behind the start of the
 *  buffer. It tests if there are <code>length</code>
 *  samples left. If there's less data, <code>length</code>
 *  will be adjusted to the remaining number of samples.
 *  \param  playCount  The current playback position.
 *  \param  len        Requested data size, will be set to number of actually remaining samples.
 *  \return A pointer to the <code>playCount</code>th sample.
 */
Uint8*  T_SoundSample::getMixableData(Uint32 playCount, Uint32 *len)
{
  Uint32 left = length - playCount;
  if (*len >= left)
  {
    *len = left;
  }
  return buffer + playCount;
}


/** \brief Get the number of bytes per sample.
 *
 *  This method returns the number of bytes per sample.
 *  For example, a 16-bit stereo sample is
 *  4 byte wide (16 bit = 2 byte per channel and two channels).
 *  \return Size of a sample in bytes
 */
int T_SoundSample::getSampleSize()
{
  return ((spec.format & 0xFF) >> 3) * spec.channels;
}


/** \brief Get the bit-depth of a sample.
 *
 *  The bit-depth represents the quality of the amplitude
 *  values of a sample.
 *  \return bit-depth of the sample
 */
int T_SoundSample::bits()
{
  return (spec.format & 0xFF);
}


/** \brief Determine if a sample is in signed or unsigned format.
 *
 *  \return true if the sample is in signed format
 */
bool T_SoundSample::isSigned()
{
  return ((spec.format & 0x8000) != 0);
}


// --- Implementation of class T_PitchVariableLoop --------
/** \brief Create a pitch variable sound loop.
 *
 *  Reads the sample from a file and sets up the dynamic
 *  buffer.
 */
T_PitchVariableLoop::T_PitchVariableLoop(const char *filename, SDL_AudioSpec *fmt)
  : T_SoundSample(filename, fmt), pitch(1.0), soundpos(0)
{
  #if DEBUG_SOUND_SERVER > 0
  printf("Reserving %d bytes dynamic sound sample buffer for %s.\n",
          fmt->samples * getSampleSize(), filename);
  #endif
  dyn_buffer.reserve(fmt->samples * getSampleSize());
}


/** \brief Deletes the sample.
 *
 *
 */
T_PitchVariableLoop::~T_PitchVariableLoop()
{
}


/** \brief Get a pointer to a chunk of data.
 *
 *  This method returns a pointer to the dynamic sample buffer,
 *  filling the dynamic buffer with interpolated sample
 *  values based on the current pitch setting. If more data
 *  is requested than the buffer can hold, the buffer will
 *  be reallocated. The value of len will therefore never
 *  change, and the sound will loop forever.
 *
 *  The method uses two kinds of arithmetic: integer-arithmetic,
 *  where all values are shifted left by the EIS constant to get
 *  (2 to the power of EIS) intermediate steps between two int
 *  values for better interpolation precision, and sample-arithmetic,
 *  where one integer step corresponds to one sample. Be careful
 *  not to perform any calculations with operands of different
 *  arithmetic without proper conversion!
 *
 *  \todo Test stereo processing with a real stereo test sound.
 *
 *  \param  playpos   The current playback position.
 *  \param  len       Requested data size.
 *  \return A pointer to the dynamic sample buffer.
 */
Uint8* T_PitchVariableLoop::getMixableData(Uint32 playpos, Uint32 *len)
{
  if (dyn_buffer.capacity() < *len)
  {
    #if DEBUG_SOUND_SERVER > 0
    printf("Reallocating dynamic sound sample buffer for %s (%d --> %d).\n",
            getName().c_str(), dyn_buffer.capacity(), *len);
    #endif
    dyn_buffer.reserve(*len);
  }
  
#if CRRC_SOUND_STEREO == 0
  // 16-bit mono samples, so we have to work through len/2 samples
  int     nSamplesToCopy = *len/2;
  Uint32  uiSoundlen = getLength()/2 << EIS;  // length in integer-arithmetic, local copy for fast access;
  Uint32  uiSoundlenSamples = getLength()/2;  // length in sample-arithmetic
#else
  // 16-bit stereo samples, so we have to work through len/4 samples
  int     nSamplesToCopy = *len/4;
  Uint32  uiSoundlen = getLength()/4 << EIS;  // length in integer-arithmetic, local copy for fast access;
  Uint32  uiSoundlenSamples = getLength()/4;  // length in sample-arithmetic
#endif  
  
  Sint16  *writeptr   = (Sint16*)&dyn_buffer[0];
  Uint32  uiSoundpos  = soundpos;         // position in integer-arithmetic (<< EIS), local copy for fast access
  Sint16* sndptr      = (Sint16*)buffer;  // local copy for fast access
  Uint32  uiPitch     = (Uint32)( (1<<EIS) * pitch); // pitch in integer-arithmetic
  while (nSamplesToCopy--)
  {
    uiSoundpos += uiPitch;
    while (uiSoundpos >= uiSoundlen)
    {
      uiSoundpos -= uiSoundlen;
    }
        
    // linear interpolation in integer arithmetic
    int    diff;
    Sint32 out_l;
    Uint32 pos1    = (uiSoundpos >> EIS);   // position, in sample-arithmetic (1 = one sample)

#if CRRC_SOUND_STEREO == 0
    Sint32 sample_l1 = *(sndptr + pos1);
#else
    Sint16 *psample = sndptr + 2 * pos1;
    Sint32 sample_l1  = *(psample);
    Sint32 sample_r1  = *(psample + 1);
    Sint32 sample_r2;
    Sint32 out_r;
#endif
    Sint32 sample_l2;
    
    if (++pos1 >= uiSoundlenSamples)
      pos1 = 0;

#if CRRC_SOUND_STEREO == 0
    sample_l2 = *(sndptr + pos1);
#else
    psample = sndptr + 2 * pos1;
    sample_l2 = *(psample);
    sample_r2 = *(psample + 1);
#endif    
    
    diff    = uiSoundpos & ((1 << EIS) - 1);
    diff = (((sample_l2 - sample_l1)*diff) >> EIS);
    out_l = sample_l1 + diff;
    
    // Limit to 16 bit samples
    if (out_l > 32767)
      out_l = 32767;
    else if (out_l < -32767)
      out_l = -32767;
     
    *writeptr++ = out_l;

#if CRRC_SOUND_STEREO == 1
    diff    = uiSoundpos & ((1 << EIS) - 1);
    diff = (((sample_r2 - sample_r1)*diff) >> EIS);
    out_r = sample_r1 + diff;
    
    // Limit to 16 bit samples
    if (out_r > 32767)
      out_r = 32767;
    else if (out_r < -32767)
      out_r = -32767;

    *writeptr++ = out_r;
#endif
  }
  soundpos = uiSoundpos;    // write back the locally changed value
  return &dyn_buffer[0];
}


/** \brief Set the pitch value for the sound loop.
 *
 *  This method controls the sample's pitch. A value
 *  of 1.0 will play the sample at the original pitch.
 *  Pitch values are automatically clamped to positive
 *  non-zero values.
 */
void T_PitchVariableLoop::setPitch(float p)
{
  SDL_LockAudio();
  if (p < 0.0001)
  {
    pitch = 0.0001;
  }
  else
  {
    pitch = p;
  }
  SDL_UnlockAudio();
}


// --- A test routine for the sound server ----------------

#ifdef STANDALONE_SOUND_TEST
CRRCAudioServer *soundserver;
#endif

#ifdef STANDALONE_SOUND_TEST
int main(int argc, char *argv[])
{
  SimpleXMLTransfer *cfgfile = new SimpleXMLTransfer("crrcsim.xml");

  printf("\n------------------------------------------\n");
  printf("Standalone test for the CRRCsim sound code\n");
  printf("------------------------------------------\n\n");
  
  /* Load the SDL library */
  printf("Initializing SDL...\n");
  if ( SDL_Init(SDL_INIT_AUDIO) < 0 )
  {
    fprintf(stderr, "Couldn't initialize SDL: %s\n",SDL_GetError());
    exit(1);
  }

  printf("Initializing sound server...\n");
  try
  {
    soundserver = new CRRCAudioServer(cfgfile);
  }
  catch (std::runtime_error& e)
  {
    fprintf(stderr, "%s\n", e.what());
    exit(0);
  }
  soundserver->pause(false);

  // play directly from file
  printf("Playing fan.wav directly from file...\n");
  soundserver->playSample("sounds/fan.wav");
  
  SDL_Delay(1000);
  T_PitchVariableLoop engine("sounds/electric.wav", soundserver->getAudioSpec());
  int engine_channel = soundserver->playSample((T_SoundSample*)&engine);
  printf("Ramping up pitch...\n");
  for (int i = 5; i < 130; i++)
  {
    float f = (float)i/40.0;
    engine.setPitch(f);
    SDL_Delay(25);
  }
  printf("Ramping down pitch...\n");
  for (int i = 130; i > 5; i--)
  {
    float f = (float)i/100.0;
    engine.setPitch(f);
    SDL_Delay(25);
  }
  engine.setPitch(1.0);
  SDL_Delay(1000);
  
  printf("Stopping sound loop...\n");
  soundserver->stopChannel(engine_channel);
  
  printf("Waiting...\n");
  SDL_Delay(2000);
  
  printf("Deleting sound server...\n");
  soundserver->pause(true);
  delete soundserver;
  
  printf("Shutting down SDL...\n");
  SDL_Quit();
  printf("Finished!\n");
  return 0;
}
#endif




