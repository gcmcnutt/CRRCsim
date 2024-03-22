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

#include "hd_ssgLOSterrain.h"


HD_SsgLOSTerrain::HD_SsgLOSTerrain(ssgRoot* SceneGraph)
: HeightData::HeightData(SceneGraph)
{
}

HD_SsgLOSTerrain::~HD_SsgLOSTerrain()
{
}

float HD_SsgLOSTerrain::getHeight(float x_north, float y_east)
{
  return getHeightAndPlane(x_north, y_east, NULL);
}

float HD_SsgLOSTerrain::getHeightAndPlane(float x_north, float y_east, float tplane[4])
{
  return HeightData::getHeightAndPlane_(x_north, y_east, NULL);
}
