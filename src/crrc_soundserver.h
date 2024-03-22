/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005, 2006, 2007, 2009 Jan Reucker (original author)
 * Copyright (C) 2005 Jan Edward Kansky
 * Copyright (C) 2005, 2008 Jens Wilhelm Wulf
 * Copyright (C) 2007 Jerry Williams
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
 *  \file crrc_soundserver.h
 *
 *  The CRRCsim sound server and related functions.
 *
 *  Author: Jan Reucker
 *  eMail:  slowhand_47@gmx.de
 */
#ifndef CRRC_SOUNDSERVER_H
#define CRRC_SOUNDSERVER_H

#include <stdexcept>
#include <string>
#include <vector>
#include <math.h>
#include <SDL.h>
#include "mod_misc/SimpleXMLTransfer.h"

/// set this to 1 to generate some debug messages
#define DEBUG_SOUND_SERVER (0)

/// maximum number of simultaneous playing samples
#define CRRC_AUDIO_CHANNELS (8) 


// Stereo sound is experimental. Right now the variometer
// is simply copied to both channels so it appears to be
// right "in the middle". The engine sound is calculated
// as a mono sound right now, using only the left channel.
#define CRRC_SOUND_STEREO   (0)   ///< 0: mono, 1: stereo

#define EIS 12      ///< integer interpolation constant


class T_SoundSample;
class CRRCAudioServer;

void PrintFormat(const char *title, SDL_AudioSpec *fmt);


/** \brief A playback container struct.
 *
 *  This container holds one sample while it is fed to
 *  the audio stream. It keeps track of all playback
 *  parameters needed by the callback.
 */
typedef struct
{
  T_SoundSample* sample;  ///< The sample to be played.
  Uint32 playpos;         ///< Current playback position.
  Uint8  volume;          ///< Playback volume of the sample.
  bool   discard;         ///< Free sample after playback has finished?
} T_PlaybackContainer;


/** \brief The sound server
 *
 *  The sound server offers a range of services to the main
 *  application:
 *    - Initializing the audio hardware
 *    - Playback of sound samples
 */
class CRRCAudioServer
{
  public:
    CRRCAudioServer(SimpleXMLTransfer *config);
    ~CRRCAudioServer();
    
    void putBackIntoConfig(SimpleXMLTransfer *config) const;

    int playSample(T_SoundSample *sample, 
                   unsigned int  volume = SDL_MIX_MAXVOLUME);
    int playSample(const char *filename,
                   unsigned int volume = SDL_MIX_MAXVOLUME);

    void stopChannel(int c);
    void stopAllChannels();

    void setChannelVolume(int c, unsigned char vol);

    SDL_AudioSpec* getAudioSpec() const;
  
    /**
     *  Get the sample rate at which the server is running.
     *  \return the current sample rate
     */
    int getSampleRate() const  {return audio_spec->freq;};
    
    /**
     *  Get the sound buffer size.
     */
    int getBufferSize() const  {return audio_spec->samples;};
  
    /** \brief Pause/resume the audio playback.
     *
     *  This method stops the audio server. All samples are
     *  halted at their current playback position.
     */
    void pause(bool do_pause = true) {int val = do_pause ? 1 : 0; SDL_PauseAudio(val); is_paused = do_pause;};
    
    /**
     *  Get a pointer to the currently running sound server instance
     *  \return pointer to the sound server
     */
    static inline CRRCAudioServer *getRunningInstance() {return instance;};
  
    friend void snd_callback(void *_unused, Uint8 *stream, int len);
    
    /**
     *  Set the volume for all model sounds.
     */
    void setModelVolume(unsigned char vol);

    /**
     *  Get the volume for all model sounds.
     *  \return model volume (0...SDL_MIX_MAXVOLUME)
     */
    unsigned char getModelVolume() const {return ucModelVolume;};

  private:
    SDL_AudioSpec*  audio_spec;       ///< the server's internal sample format
    bool            is_paused;        ///< the state of the sound server (playing or not)
    unsigned char   ucModelVolume;    ///< volume for model sounds
    T_PlaybackContainer *channel[CRRC_AUDIO_CHANNELS];  ///< the sound channels
    static CRRCAudioServer *instance;                   ///< the currently active instance

    int addSample(T_SoundSample *sample,
                                 unsigned int volume,
                                 bool disc);

};


/** \brief A sound sample.
 *
 *  A T_SoundSample object stores all data associated with
 *  a sound sample. The waveform data is read from a file
 *  and is automatically converted into the right format
 *  for playback with the CRRCAudioServer.
 *
 *  The loading and conversion mechanism is heavily
 *  inspired by SDL_mixer.
 */
class T_SoundSample
{
  public:
    T_SoundSample(SDL_AudioSpec *fmt);
    T_SoundSample(const char *filename, SDL_AudioSpec *fmt);
    virtual ~T_SoundSample();

    virtual void    convert(SDL_AudioSpec *fmt);
    virtual Uint32  getLength() const;
    virtual Uint8*  getMixableData(Uint32 playpos, Uint32 *len);

    /** 
     *  Get the sample's sample rate.
     *  \return sample rate
     */
    int getFrequency() const {return spec.freq;};
    
    /**
     *  Get the sample's sample format.
     *  \return sample format (see SDL documentation)
     */
    Uint16 getFormat() const {return spec.format;};
    
    /**
     *  Get the number of channels.
     *  \return number of channels
     */
    Uint8  getNumChannels() const {return spec.channels;};
    
    /** \brief Get the name of the sample.
     *
     *  This method returns the filename including the
     *  full path that was specified in the ctor.
     */
    std::string getName() {return samplename;};
  
   protected:
    std::string   samplename;   ///< sample filename, including full path
    SDL_AudioSpec spec;         ///< sample format
    Uint32        length;       ///< length of the sample data
    Uint8         *buffer;      ///< data buffer containing the sample data
   
    int   getSampleSize();
    int   bits();
    bool  isSigned();
};


/** \brief A variable pitch sound loop.
 *
 *  This class implements a sound loop with variable pitch.
 *  It is especially useful for engine sounds.
 */
class T_PitchVariableLoop : protected T_SoundSample
{
  public:
    T_PitchVariableLoop(const char *filename, SDL_AudioSpec *fmt);
    virtual ~T_PitchVariableLoop();
    virtual Uint8*  getMixableData(Uint32 playpos, Uint32 *len);
    virtual void    setPitch(float p);
  
  protected:
    std::vector<Uint8>  dyn_buffer;   ///< a buffer for the interpolated sample fragment
    float               pitch;        ///< current pitch
    Uint32              soundpos;     ///< current playback position in the sample
};



#endif  // CRRC_SOUNDSERVER_H
