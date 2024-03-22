/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005-2009 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2005, 2007, 2008, 2010 - Jan Reucker
 *   Copyright (C) 2005 - Lionel Cailler
 *   Copyright (C) 2006 - Todd Templeton
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
#ifndef CRRC_INPUTS_H
# define CRRC_INPUTS_H

# include <iostream>
# include <set>

#define EOM01_FIXED_Z_OFF 2.0E6

/** \brief Flight model control input struct
 *
 * Although the sim currently does only use the four basic inputs,
 * I leave the additional ones...
 */
class TSimInputs
{
  public:
  
   /**
    * To map some input for steering a wheel or something else.
    */
   enum eSteeringMap { smNOTHING, smAILERON, smELEVATOR, smRUDDER, smTHROTTLE, smFLAP, smSPOILER, smRETRACT, smPITCH };
  
   enum { NUM_AUX_INPUTS=4 };
 
   float aileron;    ///< aileron input,          -0.5 ... 0.5
   float elevator;   ///< elevator input,         -0.5 ... 0.5
   float rudder;     ///< rudder input,           -0.5 ... 0.5
   float throttle;   ///< throttle input,          0.0 ... 1.0
   float flap;       ///< flap input,              0.0 ... 1.0
   float spoiler;    ///< spoiler input,           0.0 ... 1.0
   float retract;    ///< retractable gear input,  0.0 ... 1.0
   float pitch;      ///< rotor/propeller pitch,  -0.5 ... 0.5
   float aux[NUM_AUX_INPUTS]; ///< auxiliary inputs, -0.5 ... 0.5, used in display mode
  
  /**
   * Any value < EOM01_FIXED_Z_OFF causes helicopters to stay fixed at that z-coordinate [feet].
   */
  float heli_fixed_z;
    
  /**
   * 
   */
  void CopyFrom(TSimInputs* source)
  {
    this->aileron  = source->aileron;
    this->elevator = source->elevator;
    this->rudder   = source->rudder;
    this->throttle = source->throttle;
    this->flap     = source->flap;
    this->spoiler  = source->spoiler;
    this->retract  = source->retract;
    this->pitch    = source->pitch;
    
    for(int i = 0; i < NUM_AUX_INPUTS; i++)
      this->aux[i] = source->aux[i];
    
    this->heli_fixed_z = source->heli_fixed_z;
    
    this->keys = source->keys;
  };
  
   /**
    * Returns mapped input value.
    */
   float GetInput(eSteeringMap esm) const
   {
     switch (esm)
     {
       case smAILERON:
         return(aileron);
       case smELEVATOR:
         return(elevator);
       case smRUDDER:
         return(rudder);
       case smTHROTTLE:
         return(throttle);
       case smFLAP:
         return(flap);
       case smSPOILER:
         return(spoiler);
       case smRETRACT:
         return(retract);
       case smPITCH:
         return(pitch);
       default:
         return(0);
     }
   }
  
   /**
    * maybe this does help to get some kind of random number
    */
   int getRandNum() const
   {
     int tmp = 0;

     // Just mix values in some way.
     // In this case multiplying surely leads to the result being zero.
     // What to do?
     // Add?
     // XOR?
     tmp = floatToInt(aileron);
     tmp ^= floatToInt(rudder);
     tmp ^= floatToInt(elevator);
     tmp ^= floatToInt(throttle);
     tmp ^= floatToInt(flap);
     tmp ^= floatToInt(spoiler);
     tmp ^= floatToInt(retract);
     tmp ^= floatToInt(pitch);
     
     return(tmp);
   }

   TSimInputs()
   {
     int i;
     aileron  = 0;
     elevator = 0;
     rudder   = 0;
     throttle = 0;
     flap     = 0;
     spoiler  = 0;
     retract  = 0;
     pitch    = 0;     
     for(i = 0; i < NUM_AUX_INPUTS; i++)
       aux[i] = 0;
     
     heli_fixed_z = EOM01_FIXED_Z_OFF;
   };
   
   void print()
   {
     std::cout << aileron << " " << elevator << " " << rudder << " " << throttle << "\n";
   };
  
  
  /**
   * This method is used to record keypresses not consumed by the GUI
   */
  void AddKey(int key)
  {
    keys.insert(key);
  }
  
  /**
   * This method is used to clear the list of keypresses after not being 
   * consumed by fdm or controllers.
   */
  void ClearKeys()
  {
    keys.clear();
  }
  
  /**
   * Use this method to query whether a key has been pressed. It returns 
   * true in this case and removes the key from the list, so asking a 
   * second time will return false.
   */
  bool KeyPressed(int key)
  {
    std::set<int>::iterator it;
    
    it = keys.find(key);
    if (it == keys.end())
      return(false);
    else
    {
      keys.erase(it);
      return(true);
    }
  }
  
private:
  
  /**
   * keypresses not consumed by GUI
   */
  std::set<int> keys;
  
  /**
   * inline method to convert the bit pattern of a float
   * value into an integer.
   * \param f   float value
   * \return    bit pattern of float value as integer
   */
  inline int floatToInt(float f) const
  {
    union
    {
      float f;
      int   i;
    } conv;
    conv.f = f;
    return conv.i;
  }
}; 

#endif
