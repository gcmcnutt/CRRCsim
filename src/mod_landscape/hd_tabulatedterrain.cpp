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

#include "hd_tabulatedterrain.h"


HD_TabulatedTerrain::HD_TabulatedTerrain(ssgRoot* SceneGraph)
: HeightData::HeightData(SceneGraph)
{
  make_tab_HeightAndPlane();
}

HD_TabulatedTerrain::~HD_TabulatedTerrain()
{
}

float HD_TabulatedTerrain::getHeight(float x_north, float y_east)
{
  return getHeightAndPlane(x_north, y_east, NULL);
}

float HD_TabulatedTerrain::getHeightAndPlane(float x_north, float y_east, float tplane[4])
{
  float tplane_loc[4];
  
  int i = (int)(x_north/SIZE_CELL_GRID_PLANES);
  float dx = x_north/SIZE_CELL_GRID_PLANES - i;
  i += (SIZE_GRID_PLANES/2);
  int j = (int)(y_east/SIZE_CELL_GRID_PLANES);
  float dy = y_east/SIZE_CELL_GRID_PLANES - j;
  j += (SIZE_GRID_PLANES/2);
  if (i < 0)
  {
    i = 0;
    dx = 0.0;
  }
  if (i >= SIZE_GRID_PLANES)
  {
    i = SIZE_GRID_PLANES-1;
    dx = 1.0;
  }
  if (j < 0)
  {
    j = 0;
    dy = 0.0;
  }
  if (j >= SIZE_GRID_PLANES)
  {
    j = SIZE_GRID_PLANES-1;
    dy = 1.0;
  }
  sgCopyVec4 (tplane_loc, tab_Plane[i][j]);
  if (tplane) sgCopyVec4 (tplane, tplane_loc);
  float hot =
    (1.0 - dy)*((1.0 - dx)*tab_HOT[i][j]  + dx*tab_HOT[i+1][j]  )
    +      dy *((1.0 - dx)*tab_HOT[i][j+1]+ dx*tab_HOT[i+1][j+1]);
  return hot;
}

void HD_TabulatedTerrain::make_tab_HeightAndPlane()
{
  for (int i = 0; i <= SIZE_GRID_PLANES; i++)
  {
    for (int j = 0; j <= SIZE_GRID_PLANES; j++)
    {
      float x = (i - SIZE_GRID_PLANES/2)*SIZE_CELL_GRID_PLANES;
      float y = (j - SIZE_GRID_PLANES/2)*SIZE_CELL_GRID_PLANES;
      float h = HeightData::getHeightAndPlane_(x, y, tab_Plane[i][j]);
      tab_HOT[i][j] = h;
    }
  }
}
