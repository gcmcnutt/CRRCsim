/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2005, 2008-2009 - Jens Wilhelm Wulf (original author)
 *   Copyright (C) 2008 - Jan Reucker
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
#ifndef PT1_H
# define PT1_H

namespace CRRCMath
{
  /**
   * PT1-Glied
   *
   * @author Jens Wilhelm Wulf
   */
  class PT1
  {
    public:

     /**
      * @param ival             initial value
      * @param zeitkonstante    zero means no filter at all
      */
     void init(double ival, double zeitkonstante);

     /**
      * @param ival initial value
      */
     void init(double ival);

     /**
      * @param zeitkonstante zero means no filter at all
      */
     void SetTau(double zeitkonstante);

     /**
      * @param dt   timestep [s]
      * @param nval new input value
      */
     void step(double dt, double nval);

     /**
      * Value of filter output
      */
     double val;

    private:
     double dZeitkonstante;
     double d_Mul;
     double d_dt_Alt;
  };
}
#endif
