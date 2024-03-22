/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 *   Copyright (C) 2009 Joel Lienard (original author)
 *   Copyright (C) 2009 Jan Reucker
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
 * This file can optionally be linked with the CGAL library
 * (http://www.cgal.org). Parts of this library are licensed
 * under the QPL which is incompatible to the GPL. Therefore,
 * as a special exception, you have permission to link this program
 * with the CGAL library and distribute executables, as long as you
 * follow the requirements of the GNU GPL in regard to all of the
 * software in the executable aside from CGAL.
 *
 */

#ifndef WINDDATA3D_H
#define WINDDATA3D_H

#if WINDDATA3D == 1
#include <vector>
#include <list>
#include <plib/sg.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>

int init_wind_data(char *);
int find_wind_data(float,float,float,float*,float*,float*);
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_vertex_base_with_info_3<sgVec3, K> Vb;
typedef CGAL::Triangulation_data_structure_3<Vb>                    Tds;
typedef CGAL::Delaunay_triangulation_3<K, Tds>                      WindData;

typedef WindData::Point   Point;
typedef Tds::Vertex_handle  Vertex_handle; 
typedef Tds::Cell_handle  Cell_handle; 

#endif // WINDDATA3D

#endif // WINDDATA3D_H
