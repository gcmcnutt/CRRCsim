/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 *   Copyright (C) 2009 Jan Reucker (original author)
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

#include "heightdata.h"


HeightData::HeightData(ssgRoot* SceneGraph)
{
  SceneGraph_ = SceneGraph;
}

float HeightData::getHeightAndPlane_(float x_north, float y_east, float tplane[4])
{
  ssgHit *results;
  int num_hits;
  float hot;   /* H.O.T == Height Of Terrain */
  sgVec3 s;
  sgSetVec3(s, 0.0, 1.0, 0.0);
  sgMat4 m = {  {  1,   0,   0,   0},
                {  0,   1,   0,   0},
                {  0,   0,   1,   0},
                {-y_east , 0, x_north, 1.0}
              };
  num_hits = ssgLOS(SceneGraph_, s, m, &results );
  hot = DEEPEST_HELL;
  int numero = -1;
  for ( int i = 0 ; i < num_hits ; i++ )
  {
    ssgHit *h = &results [ i ];
    float hgt = - h->plane[3] / h->plane[1];
    if (hgt >= hot)
    {
      hot = hgt;
      numero = i;
    }
  }
  if ( tplane )
  {
    if (numero >= 0)
    {
      sgCopyVec4 ( tplane, results [ numero ].plane );
      tplane[3] = tplane[3] - tplane[0]*y_east + tplane[2]*x_north;
      if (tplane[1] < 0) /*  ??? revoir :  preferable d'orienter correctement les facettes */
        sgNegateVec4 ( tplane , tplane );
      //std::cout << "----plane***" <<  tplane[0] <<"  "<<  tplane[1] <<"  "<<  tplane[2]<<"  "<<  tplane[3] <<std::endl;
    }
    else
    {
      tplane[0] = 0.0;
      tplane[1] = 1.0;
      tplane[2] = 0.0;
      tplane[3] = -hot;
    }
  }
  return hot;
}
