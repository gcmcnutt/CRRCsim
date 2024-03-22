/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2006, 2008 Jens Wilhelm Wulf (original author)
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

#ifndef CRRC_RAND
#define CRRC_RAND

#include <stdlib.h>

/**
 * This is just a wrapper around standard functions rand and srand which
 * allows for easy re-seeding.
 * 
 * From libc manual:
 * 
 * All functions have in common that they use the same congruential formula 
 * with the same constants. The formula is
 * 
 *    Y = (a * X + c) mod m
 *
 * where X is the state of the generator at the beginning and Y the state at
 * the end. a and c are constants determining the way the generator works.
 * By default they are
 *
 *     a = 0x5DEECE66D = 25214903917
 *     c = 0xb = 11
 *
 * From man rand:
 *  POSIX 1003.1?2003 gives the following example of an implementation of 
 *  rand() and srand(), possibly useful when one needs the same sequence
 *  on two different machines.
 *
 *       static unsigned long next = 1;
 *       * 
 *       // RAND_MAX assumed to be 32767 
 *       int myrand(void) {
 *       next = next * 1103515245 + 12345;
 *       return((unsigned)(next/65536) % 32768);
 *       }
 * 
 * @author Jens Wilhelm Wulf
 */
class CRRC_Random
{
  public:
   
   /**
    * Returns a random number between 0 and max().
    */
   static inline int rand() { return(::rand()); };

   static inline int max() { return(RAND_MAX); };
   
   /**
    * Call this using some random data you have, anytime you want to.
    */
   static void insertData(int nData);
   
  private:
   
   /**
    * random state
    */
   static unsigned int uRandState16;
   static unsigned int uRandState32;
   
};

/**
 * Returns random numbers with normal (gaussian) distribution, 
 * possibly of specified sigma and average.
 *
 * Thanks to rhoads@paul.rutgers.edu for this code!
 * http://remus.rutgers.edu/~rhoads/Code/code.html
 * 
 * @author Jens W. Wulf
 */
class RandGauss
{
  public:
  
    RandGauss();
    
    RandGauss( double sigma, double mean );

    inline void SetSigma( double sigma )
      { sigma_ = sigma; };

    inline void SetMean( double mean )
      { mean_ = mean; };

    inline void SetSigmaAndMean( double sigma, double mean )
      { sigma_ = sigma; mean_ = mean; };
    
    double Get();
    
  private:
  
    double sigma_, mean_, V2, fac;
    int phase;
};

#endif

