/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2005-2009 Jan Reucker (original author)
 * Copyright (C) 2005, 2006, 2008 Jens Wilhelm Wulf
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

#ifndef CRRC_LOADAIR_H
# define CRRC_LOADAIR_H

# include <string>
# include <vector>

#include "global.h"
#include "global_video.h"
#include "include_gl.h"
#include "mod_fdm/fdm.h"
#include "crrc_sound.h"
#include "mod_misc/SimpleXMLTransfer.h"



/** \brief A very basic airplane description class.
 *
 *  This class provides an abstract interface for everything
 *  related to the audio-visual representation of an airplane
 *  model.
 */
class CRRCAirplane
{
  public:
   
   /** \brief The constructor.
    *
    *  
    */
   CRRCAirplane();
   
   /** \brief The destructor.
    *
    *  Clean up.
    */
   virtual ~CRRCAirplane();
   
   /** \brief Update the airplane visualisation
    *
    *
    */
   virtual void update(FDMBase* airplane) = 0;

   /** \brief Draw the airplane visualisation
    *
    *
    */
   virtual void draw() = 0;

   /** \brief Reset the airplane visualisation
    *
    *
    */
   virtual void reset(FDMBase* airplane) = 0;

   virtual int getNumSounds()  {return (sound.size());};
 
   virtual float getAircraftSize()  {return (airplaneRadius);};
 
  protected:
   std::vector<T_AirplaneSound*> sound;
   float airplaneRadius;
};


/**
 * Read the audio-visual airplane description from a
 * CRRCSim_airplane XML file, file format version "2".
 */
class CRRCAirplaneV2 : public CRRCAirplane
{
  public:
    CRRCAirplaneV2();
    CRRCAirplaneV2(SimpleXMLTransfer* xml);
    ~CRRCAirplaneV2();

    void update(FDMBase* airplane);
    void draw();
    void reset(FDMBase* airplane);
  
  private:
    long lVisID;    ///< ID for the airplane visualization

  protected:
   
  /** \brief Initialize the airplane's sound.
    *
    *  Reads all sound related parameters from an xml description.
    *  \todo Make this method search all possible paths on Linux!
    */
   virtual void  initSound(SimpleXMLTransfer* xml);

   float max_thrust;
};



#endif // CRRC_LOADAIR_H
