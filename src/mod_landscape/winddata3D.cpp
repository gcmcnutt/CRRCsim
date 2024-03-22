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

/** \file winddata3D.cpp
 *   read datafile of wind velocity :
 is a text file with 6 numbers per line, blank separated  : X Y Z vx vy vz
 X,Y,Z coordinates, in meter ou feet (see "units" in scenery xml file)
 vx, vy, vz wind vector components,  normalised to 1
 */

#include <crrc_config.h>

#include "crrc_scenery.h"
#include "model_based_scenery.h"
#if WINDDATA3D == 1
#if CGAL_VERSION3 == 1
# include <CGAL/assertions.h>
#else
# include <CGAL/assertions_behaviour.h> 
#endif
#ifdef TEST_WINDDATA
//main program for test only
WindData *wind_data=NULL;
main()
{
//init
  int r= init_wind_data("scenery/Brie/wind.data");
  std::cout << r << "  points processed" << std::endl;
//std::cout <<T;
  float x,y,z,vx,vy,vz;
  int  ok;
  do
  {
    std::cout <<std::endl<<"coodonnées à tester (x y z) ?"<<std::endl;
    scanf("%f",&x);
    scanf("%f",&y);
    scanf("%f",&z);
    ok = find_wind_data(x,y,z,&vx,&vy,&vz);
    if (ok)
    {
      std::cout<<"find:  " << vx << "  "<< vy << "  " << vz <<std::endl;
    }
    else
    {
      std::cout<<"Hors table"<<std::endl;
    }

  }
  while (1);
}
#endif
//////////////////

#ifdef TEST_WINDDATA
int init_wind_data(char * filename)
#else
int ModelBasedScenery::init_wind_data(const char* filename)
#endif
{
  CGAL::set_error_behaviour ( CGAL::CONTINUE); //CGAL failure behaviour
  Vertex_handle v;
  FILE *input = fopen(filename,"r");
  float north, east, up, v_north, v_east, v_up;
  int npt=0;
  int nread;
  int stop=0;
  if (!input) fprintf(stderr, "Error open wind filename:  %s \n",filename);
  else
#if 1
    {
     wind_data = new WindData;    
     do
     {
      nread = fscanf(input,"%f %f %f %f %f %f",&north, &east, &up, &v_north, &v_east, &v_up);
      if (nread==6)
      {
        v = wind_data->insert(Point(north, east, up));
        sgSetVec3(v->info(),v_north,v_east,v_up);
        npt++;
      }
      else stop=1;
     }
     while (!stop);
    }
#else
  {//essai d'acceleration : efficace mais comment ajouter les info  ???
    //voir "2.5   Extensible Kernel" in
    //    http://www.cgal.org/Manual/3.3/doc_html/cgal_manual/Kernel_23/Chapter_main.html#Section_2.2
    std::list<Point> L;
    do
    {
      nread = fscanf(input,"%f %f %f %f %f %f",&north, &east, &up, &v_north, &v_east, &v_up);
      if (nread==6)
      {
        L.push_front(Point(north, east, up));
        //sgSetVec3(v->info(),v_north,v_east,v_up);
        npt++;
      }
      else stop=1;
    }
    while (!stop);
    wind_data->insert(L.begin(), L.end());
    L.clear();
  }
#endif
  return npt;
}
void Point_sgVec3(Point p, sgVec3 v)
{
  v[0]=p.x();
  v[1]=p.y();
  v[2]=p.z();
}
float cbary(Point pb , Point pa, Point pc, Point pd)
{
  sgVec3 a, b ,c ,d ,ba ,bc ,bd ,v;
  float res;
  Point_sgVec3(pa, a);
  Point_sgVec3(pb, b);
  Point_sgVec3(pc, c);
  Point_sgVec3(pd, d);
  sgSubVec3  ( ba, a, b );
  //std::cout <<"cbary: ba : "<< ba[0] <<" "<< ba[1] <<" "<< ba[2] <<std::endl  ;
  sgSubVec3  ( bc, c, b );
  sgSubVec3  ( bd, d, b );
  sgVectorProductVec3 ( v, bc, bd );
  res = sgScalarProductVec3 ( v, ba );
  if (res<0.)res = -res; //abs value
  return res;
}

#ifdef TEST_WINDDATA
int find_wind_data(float n,float e,float u, float *vx, float *vy, float * vz)
#else
int ModelBasedScenery::find_wind_data(float n,float e,float u, float *vx, float *vy, float * vz)
#endif
{
  static Cell_handle mem_cell_handle=NULL;
  Cell_handle c;
  Point p0 = Point(n,e,u);
  if (mem_cell_handle==NULL) c = wind_data->locate(p0,mem_cell_handle);
  else c = wind_data->locate(p0);
  mem_cell_handle = c;
  if (wind_data->is_infinite (c)) return false;
  Vertex_handle va = c->vertex(0);
  Vertex_handle vb = c->vertex(1);
  Vertex_handle vc = c->vertex(2);
  Vertex_handle vd = c->vertex(3);
  Point pa = va->point();
  Point pb = vb->point();
  Point pc = vc->point();
  Point pd = vd->point();

  //std::cout <<"Points trouvés: "<< pa <<std::endl<< pb <<std::endl<< pc <<std::endl << pd <<std::endl;
  float *wa = va->info();
  float *wb = vb->info();
  float *wc = vc->info();
  float *wd = vd->info();
  /*
  std::cout <<"Valeurs trouvés: "<< wa[0] <<" "<< wa[1] <<" "<< wa[2] <<std::endl  ;
  std::cout <<"               : "<< wb[0] <<" "<< wb[1] <<" "<< wb[2] <<std::endl  ;
  std::cout <<"               : "<< wc[0] <<" "<< wc[1] <<" "<< wc[2] <<std::endl  ;
  std::cout <<"               : "<< wd[0] <<" "<< wd[1] <<" "<< wd[2] <<std::endl  ;
  */
  float c0,ca,cb,cc,cd;
  c0 = cbary(pa,pb,pc,pd);
  ca = cbary(p0,pb,pc,pd) / c0;
  cb = cbary(p0,pc,pd,pa) / c0;
  cc = cbary(p0,pd,pa,pb) / c0;
  cd = cbary(p0,pa,pb,pc) / c0;
  /*
  std::cout <<"coef 0: "<<c0<< std::endl;
  std::cout <<"coef a: "<<ca<< std::endl;
  std::cout <<"coef b: "<<cb<< std::endl;
  std::cout <<"coef c: "<<cc<< std::endl;
  std::cout <<"coef d: "<<cd<< std::endl;
  std::cout <<"somme coefs: "<<(ca+cb+cc+cd)<< std::endl;// verif
  */
  sgVec3 resul;
  sgScaleVec3  ( resul, wa, ca );
  sgAddScaledVec3  ( resul, wb, cb );
  sgAddScaledVec3  ( resul, wc, cc );
  sgAddScaledVec3  ( resul, wd, cd );
  *vx = resul[0];
  *vy = resul[1];
  *vz = resul[2];
  return true;
}
#endif
