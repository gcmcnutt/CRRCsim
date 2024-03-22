/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2004 Kees Lemmens (original author)
 * Copyright (C) 2004 Lionel Cailler
 * Copyright (C) 2005 Jan Edward Kansky
 * Copyright (C) 2005, 2006, 2008 Jens Wilhelm Wulf
 * Copyright (C) 2005 - 2009 Jan Reucker
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
  

/** \file crrc_sound.cpp
 * 
 *  CRRCsim-specific stuff related to the soundserver.
 */

#include <math.h>

#include "crrc_main.h"
#include "crrc_sound.h"
#include "crrc_soundserver.h"
#include "crrc_loadair.h"


#define SPEED_OF_SOUND    ((float)(330.0 / 0.3048))    // ft/s
#define VARIO             (2)   // 1: test, 2: real vario

/** \brief Structure for exchanging 3D data between main thread and sound thread.
 *
 *  The data in this structure will be filled in by soundUpdate3D().
 *  The sound thread uses it to calculate the absolute pitch value and
 *  volume setting for playing the sample.
 */
struct tagAudio3D
{
  float flPropFreq;     ///< pitch input from Tx, 0.0 ... 1.0
  float flDist;         ///< current distance to model
  float flH;            ///< current altitude
  float flRelVelocity;  ///< relative velocity
} Audio3D;



// --- Implementation of class T_AirplaneSound
T_AirplaneSound::T_AirplaneSound(const char *filename, SDL_AudioSpec *fmt)
  : T_PitchVariableLoop(filename, fmt),
    dPitchFactor(0.002), dMaxVolume(1.0), 
    type(SOUND_TYPE_GLIDER), channel(-1),
    last_dist(0), v_rel_filter(0)
{
}

T_AirplaneSound::~T_AirplaneSound()
{
}

/** \brief Get a pointer to a chunk of data.
 *
 *  This method calculates the interpolated engine sound
 *  samples. The value of len will never
 *  change, and the sound will loop forever.
 *
 *  \param  playpos   The current playback position.
 *  \param  len       Requested data size.
 *  \return A pointer to the dynamic sample buffer.
 */
Uint8* T_AirplaneSound::getMixableData(Uint32 playpos, Uint32 *len)
{
  calculate();
  Uint8* ret = T_PitchVariableLoop::getMixableData(playpos, len);
  return ret;
}


/** \brief Calculate the pitch shift caused by the Doppler effect
 *
 *  The movement of a sound source relative to the listener causes
 *  a pitch shift called the Doppler effect. If the sound source
 *  approaches the listener, the sound frequency seems to rise,
 *  and vice versa. The Doppler effect can be calculated using
 *  this formula:
 *
 *  \verbatim
 *               f_source       f_list:   frequency as heard by listener
 *   f_list =  -------------    f_source: sound source frequency
 *             (1 + v_rel/c)    v_rel:    sound source speed relative to listener
 *                              c:        speed of sound
 *  \endverbatim
 *
 *  Divide the sound source frequency by the Doppler constant to
 *  get the frequency as heard by the listener. Please note that
 *  the relative velocity will be filtered internally to compensate
 *  the poor resolution of the timestep values (especially when
 *  using a low DAC sampling rate).
 *
 *  \param dist Current distance to the sound source
 *  \return Doppler constant (1 + v_rel/c)
 */
float T_AirplaneSound::calculate_Doppler(float dist)
{
  CRRCAudioServer *server = CRRCAudioServer::getRunningInstance();

  float         flSndTimeDiff = (float)server->getBufferSize() 
                                / (float)server->getSampleRate();
  float         v_rel;
  float         C_doppler;
    
  v_rel    = (dist - last_dist) / flSndTimeDiff;
  // simple filter for velocity values to compensate poor resolution
  // of timestep values.
  v_rel_filter += (v_rel - v_rel_filter) / 2;
  last_dist = dist;

  C_doppler = 1.0 + (v_rel_filter / SPEED_OF_SOUND);
    
  return C_doppler;
}



// --- Implementation of class T_EngineSound --------------

T_EngineSound::T_EngineSound(const char *filename, SDL_AudioSpec *fmt)
  : T_AirplaneSound(filename, fmt)
{
}


T_EngineSound::~T_EngineSound()
{
}


#define VOLUME_ATT_MAX_DISTANCE_FT    1800
#define VOLUME_ATT_MIN_VOLUME         7


/** \brief Calculate pitch and volume for the engine sample
 *
 *
 */
void T_EngineSound::calculate()
{
  CRRCAudioServer *server = CRRCAudioServer::getRunningInstance();

  unsigned char nEngineVol;
  float flPropFreq;
  float flDist;
  float C_doppler;
  float flModelVolume;

  // get input values from inter-process swap buffer
  flPropFreq    = Audio3D.flPropFreq;
  flDist        = Audio3D.flDist;
  flModelVolume = (float)server->getModelVolume() / (float)SDL_MIX_MAXVOLUME;

  C_doppler = calculate_Doppler(flDist);
  
  // Distance-dependend attenuation:
  if (flPropFreq < 0.001)
  {
    nEngineVol = 0;
  }
  else
  {
    if (flDist > VOLUME_ATT_MAX_DISTANCE_FT)
      flDist = VOLUME_ATT_MAX_DISTANCE_FT;
    
    float fEngineVol = (float)SDL_MIX_MAXVOLUME 
                        - ((float)SDL_MIX_MAXVOLUME * flDist / VOLUME_ATT_MAX_DISTANCE_FT);
    nEngineVol = (unsigned char)(fEngineVol * dMaxVolume * flModelVolume);

    // Make sure that the sound never dies (this would sound
    // really unrealistic)
    if (nEngineVol < VOLUME_ATT_MIN_VOLUME)
    {
      nEngineVol = VOLUME_ATT_MIN_VOLUME;
    }
  }
  float pitch = 0.8 * flPropFreq*dPitchFactor / C_doppler;
  setPitch(pitch);
  server->setChannelVolume(channel, nEngineVol);
}


// --- Implementation of class T_GliderSound --------------
T_GliderSound::T_GliderSound(const char *filename, SDL_AudioSpec *fmt)
  : T_AirplaneSound(filename, fmt),
    flMinRelV(1.5), flMaxRelV(4.0), flMaxDist(300)
{
}


T_GliderSound::~T_GliderSound()
{
}


/** \brief Calculate pitch and volume for the glider sample
 *
 *
 */
void T_GliderSound::calculate()
{
  unsigned char   nSoundVol;
  float           fSoundVol;
  CRRCAudioServer *server = CRRCAudioServer::getRunningInstance();

  // get input values from inter-process swap buffer
  float flDist        = Audio3D.flDist;
  float flRelV        = Audio3D.flRelVelocity;
  float flModelVolume = (float)server->getModelVolume() / (float)SDL_MIX_MAXVOLUME;;

  // calculate Doppler effect
  float C_doppler = calculate_Doppler(flDist);
  
  // Distance-dependend attenuation:
  // Volume decreases linearly until it reaches 5% at a distance
  // of flMaxDist, then it stays at 5 %
  if (flDist > flMaxDist)
    flDist = flMaxDist;
  
  fSoundVol = (float)SDL_MIX_MAXVOLUME 
              - ((float)SDL_MIX_MAXVOLUME * 0.95 * flDist / flMaxDist);
  
  // Speed-dependend attenuation:
  // Below the minimum relative velocity, the volume
  // drops to zero. At flMaxRelV times the trimmed flight velocity, 
  // the volume reaches the maximum.
  if (flRelV < flMinRelV)
  {
    fSoundVol = 0.0;
  }
  else if (flRelV < flMaxRelV)
  {
    fSoundVol *= (flRelV - flMinRelV) / (flMaxRelV - flMinRelV);
  }
  
  nSoundVol = (unsigned char)(fSoundVol * dMaxVolume * flModelVolume);
  
  float pitch = 0.2 + 0.8 * dPitchFactor / C_doppler;
  setPitch(pitch);
  server->setChannelVolume(channel, nSoundVol);
}


// --- Implementation of class T_VariometerSound ----------
/**
 * Variometer: max climb value in ft/s
 */
const float flSndVarioClimbMax = (3.0 / 0.3048);

/**
 * Variometer: max sink value in ft/s
 */
const float flSndVarioSinkMax = (1.5 / 0.3048);

/**
 * Variometer: deadband ft/s
 */
const float flSndVarioDeadband = (0.0 / 0.3048);

/**
 * Variometer: max frequency in Hz
 */
const float flSndVarioFMax = 4000;

/**
 * Variometer: min frequency in Hz
 */
const float flSndVarioFMin = 100;

/**
 * Variometer: min frequency (climb) in Hz
 */
const float flSndVarioFClimbMin = 600;

/**
 * Variometer: max frequency (sink) in Hz
 */
const float flSndVarioFSinkMax = 400;

/**
 * Variometer: min length of beep in seconds
 */
const float flSndVarioDurMin = 0.1;

/**
 * Variometer: max length of beep in seconds
 */
const float flSndVarioDurMax = 0.5;

//const float flSndServerMaxSamplerate = 48000;


/** \brief Create the variometer sound object.
 *
 *  \param fmt desired sound format
 */
T_VariometerSound::T_VariometerSound(SDL_AudioSpec *fmt)
  : T_SoundSample(fmt), nBeepFInc(0), nBeepFIncNew(0), nOnCntInit(0),
    nVarioCnt(0), nSinIndex(0), flHDiffFilt(0.0), flHOld(0.0)
{
  init(fmt);
  samplename = "variometer_sound";
}


/** \brief Initialize the object's internal data
 *
 *  \param fmt desired sound format
 */
void T_VariometerSound::init(SDL_AudioSpec *fmt)
{
  // init sine
  nSndVarioSineLen = (int)(fmt->freq / flSndVarioFMin);
  sinewave.reserve(nSndVarioSineLen);
  dyn_buffer.reserve(fmt->samples * getSampleSize());

  for (int n = 0; n < nSndVarioSineLen; n++)
  {
    sinewave[n] = (Sint16)(32767 * sin(2*M_PI*n/nSndVarioSineLen));
  }
  printf("Initialized sine samples: %i\n", nSndVarioSineLen);

}


/** \brief Convert to different sound parameters.
 *
 *  Replaces the base-class' convert() method with the
 *  vario-initialization.
 */
void T_VariometerSound::convert(SDL_AudioSpec *fmt)
{
  init(fmt);
}


/** \brief Delete the variometer sound object.
 *
 *
 */
T_VariometerSound::~T_VariometerSound()
{
}


/** \brief Calculate the vario parameters
 *
 *  Calculates all parameters needed for the next
 *  chunk of data to be mixed into the sound stream.
 *
 *  \todo Remove references to soundserver
 */
void T_VariometerSound::calculate()
{
  float         flHIn;
  float         flSndTimeDiff = (float)Global::soundserver->getBufferSize() 
                                / (float)Global::soundserver->getSampleRate();
  // get input values from inter-process swap buffer
  flHIn       = Audio3D.flH;
  
  // feet per second
  float flHDiff = (flHIn - flHOld) / flSndTimeDiff;
  flHOld = flHIn;

  // todo: time constant
  flHDiffFilt += (flHDiff - flHDiffFilt) / 4;
  
  // frequency */
  float flF;
  float flVal;

  if (flHDiffFilt > flSndVarioDeadband)
  {
    flVal = flHDiffFilt / flSndVarioClimbMax;
    if (flVal > 1)
      flVal = 1;
    flF = flSndVarioFClimbMin + (flSndVarioFMax-flSndVarioFClimbMin) * flVal;

    // flVal = 0 -> flDur = flSndVarioDurMax
    // flVal = 1 -> flDur = flSndVarioDurMin
    float flDur = flSndVarioDurMax - flVal * (flSndVarioDurMax-flSndVarioDurMin);
          
    nOnCntInit    = (int)(flDur * Global::soundserver->getSampleRate());
    nBeepFIncNew  = (int)((flF*256) / flSndVarioFMin);
  }
  else if (flHDiffFilt < -1*flSndVarioDeadband)
  {
    flVal = flHDiffFilt / flSndVarioSinkMax;
    if (flVal < -1)
      flVal = -1;
    flF = flSndVarioFSinkMax + (flSndVarioFSinkMax-flSndVarioFMin) * flVal;

    nOnCntInit = 0;
    nBeepFInc  = (int)((flF*256) / flSndVarioFMin);
  }
  else
  {
    nVarioOffReq = 1;
  }    
}


/** \brief Get a pointer to a chunk of data.
 *
 *  This method calculates the dynamic variometer sound
 *  samples. The value of len will never
 *  change, and the sound will loop forever.
 *
 *  \param  playpos   The current playback position.
 *  \param  len       Requested data size.
 *  \return A pointer to the dynamic sample buffer.
 */
Uint8* T_VariometerSound::getMixableData(Uint32 playpos, Uint32 *len)
{
  Sint16*     writeptr;
  int         nVarioDiff;
  Sint32      nSample;

  calculate();

#if CRRC_SOUND_STEREO == 0
  // 16-bit mono samples, so we have to work through len/2 samples
  int nSamplesToCopy = *len/2;
#else
  // 16-bit stereo samples, so we have to work through len/4 samples
  int nSamplesToCopy = *len/4;
#endif  
  
  writeptr = (Sint16*)&dyn_buffer[0];
    
  while (nSamplesToCopy--)
  {
#if VARIO == 2    
    nVarioDiff = sinewave[nSinIndex>>8];      
    nSinIndex += nBeepFInc;

    if (nVarioCnt > 0)
      nVarioCnt--;
    
    if (nVarioCnt == 0 && nBeepFInc == 0 && nOnCntInit != 0)
    {
      nBeepFInc = nBeepFIncNew;
      nVarioCnt = nOnCntInit;
    }
    
    if (nSinIndex >= (nSndVarioSineLen<<8))
    {
      if (nVarioOffReq)
      {
        nVarioOffReq = 0;
        nBeepFInc    = 0;
        nSinIndex    = 0;
      }
      else if (nVarioCnt == 0 && nOnCntInit != 0)
      {
        if (nBeepFInc)
        {
          nBeepFInc    = 0;
          nSinIndex    = 0;
          nVarioCnt    = nOnCntInit;
        }
      }
      else
        nSinIndex -= (nSndVarioSineLen<<8);
    }
#else
    nVarioDiff = sinewave[nSinIndex>>8];
    nSinIndex += 16<<8;
    while (nSinIndex >= (nSndVarioSineLen<<8))
      nSinIndex -= (nSndVarioSineLen<<8);
    nSndVarioVol = 9;
#endif
            
    // Mix
    nSample = nVarioDiff << 1;
    
    // Limit to 16 bit samples
    if (nSample > 32767)
      nSample = 32767;
    else if (nSample < -32767)
      nSample = -32767;
    *writeptr++ = (Sint16)nSample;
#if CRRC_SOUND_STEREO == 1
    // write the same value to the second stereo channel
    // (vario and engine will be "straight ahead")
    *writeptr++ = (Sint16)nSample;
#endif    
  }
  return (Uint8*)&dyn_buffer[0];
}



/** \brief Update the 3DSound input data.
 *
 *  Writes the current flight model values to the exchange buffer.
 *
 *  \param dist           Distance from listener to model
 *  \param dPropFreq      airplane propeller frequency
 *  \param flH            airplane altitude
 *  \param flRelV         airplane velocity relative to trimmed velocity
 */
void soundUpdate3D(float flDist, float flPropFreq, float flH, float flRelV)
{
  SDL_LockAudio();
  Audio3D.flPropFreq    = flPropFreq;
  Audio3D.flDist        = flDist;
  Audio3D.flH           = flH;
  Audio3D.flRelVelocity = flRelV;
  SDL_UnlockAudio();
}


