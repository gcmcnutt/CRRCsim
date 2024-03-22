/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2008 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2010 - Jan Reucker (conversion to template class)
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
#ifndef RATELIM_H
# define RATELIM_H

namespace CRRCMath
{
  /**
   * Limits the rate of change of a value.
   *
   * @author Jens Wilhelm Wulf
   */
  template <typename T>
  class RateLimiter
  {
    public:
      /**
       * @param ival      initial value
       * @param rate      rate of change per second
       */
      RateLimiter(T ival = 0, T rate = 1)
      {
        init(ival, rate);
      }
     
      /**
       * @param ival      initial value
       * @param rate      the value is allowed to change rate per second
       */
      void init(T ival, T rate)
      {
        val     = ival;
        ratemax = rate;
      }

      /**
       * @param dt   timestep [s]
       * @param nval new input value
       */
      void step(double dt, T nval)
      {
        T dmax = ratemax * dt;

        T diff = nval - val;

        if (diff > dmax)
        {
          diff = dmax;
        }
        else if (diff < -dmax)
        {
          diff = -dmax;
        }

        val += diff;
      }


      /**
       * limited value
       */
      T val;

    private:
      T ratemax;
  };
};
#endif
