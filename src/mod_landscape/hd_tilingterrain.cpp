/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2009 Jan Reucker (original author)
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

#include "hd_tilingterrain.h"


HD_TilingTerrain::HD_TilingTerrain(ssgRoot* SceneGraph)
: HeightData::HeightData(SceneGraph)
{
  for ( int i = 0 ; i < SIZE_GRID_PLANES; i++ )
  {
    for ( int j = 0 ; j < SIZE_GRID_PLANES; j++ )
    {
      tile_table[i][j] = new ssgVertexArray();
    }
  }

  sgMat4 xform;
  sgMakeIdentMat4(xform);
  tiling_terrain(SceneGraph,xform);
}

HD_TilingTerrain::~HD_TilingTerrain()
{
  for ( int i = 0 ; i < SIZE_GRID_PLANES; i++)
  {
    for ( int j = 0 ; j < SIZE_GRID_PLANES; j++)
    {
      if (tile_table[i][j])
        delete tile_table[i][j];
    }
  }
}

float HD_TilingTerrain::getHeight(float x_north, float y_east)
{
  return getHeightAndPlane(x_north, y_east, NULL);
}

float HD_TilingTerrain::getHeightAndPlane(float x_north, float y_east, float tplane[4])
{
  int numero = -1;

  int i = (int)(y_east/SIZE_CELL_GRID_PLANES) + SIZE_GRID_PLANES/2;
  int j = (int)(-x_north/SIZE_CELL_GRID_PLANES) + SIZE_GRID_PLANES/2;
  if (i < 0) i = 0;
  if (i > SIZE_GRID_PLANES-1) i = SIZE_GRID_PLANES-1;
  if (j < 0) j = 0;
  if (j > SIZE_GRID_PLANES-1) j = SIZE_GRID_PLANES-1;
  ssgVertexArray* tile = tile_table[i][j];
//std::cout << "utilise cellule " << i<<", "<<j<< std::endl;

  int nt = tile->getNum();
  float hot = DEEPEST_HELL ;
//std::cout << "cellule nb triangle/cellule*3 i,j "<<tile <<"  " << nt <<"  "<< i <<"  "<< j << std::endl;
//std::cout << "nord, east" <<x_north<<"  "<< y_east <<"  " << std::endl;

  for ( int n = 0 ; n < nt ; n+=3 )
  {
    float *p1 = tile->get(n);
    float *p2 = tile->get(n+1);
    float *p3 = tile->get(n+2);
    /*
    std::cout << "-------------- " << p1[0]<<"  "<< p1[1]<<"  "<< p1[2]<<"  "<< std::endl;
    std::cout << "-------------- " << p2[0]<<"  "<< p2[1]<<"  "<< p2[2]<<"  "<< std::endl;
    std::cout << "-------------- " << p3[0]<<"  "<< p3[1]<<"  "<< p3[2]<<"  "<< std::endl;
    */
    if ( on_triangle(-x_north, y_east, p1, p2, p3))
    {
      sgVec4 plane;
      sgMakePlane ( plane, p1, p2, p3);
      float h = -(-plane[2]*x_north + plane[0]*y_east + plane[3])/plane[1];
      if (h > hot)
      {
        numero = n;
        hot = h;
        if (tplane) sgCopyVec4(tplane, plane);
      }
      //std::cout << "----plane***" <<  plane[0] <<"  "<<  plane[1] <<"  "<<  plane[2]<<"  "<<  plane[3] << "hot" << hot <<std::endl;
    }
  }
  if ( tplane )
  {
    if ( numero >= 0)
    {
      if (tplane[1]<0) /*  ??? revoir :  preferable d'orienter correctement les facettes */
        sgNegateVec4 ( tplane , tplane );
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

/**
 * \brief Tile the terrain
 *
 * This function recursively walks the scene graph and sorts all
 * triangles into a grid of smaller graphs. This reduces the
 * calculation effort: If the position of the plane and therefore
 * the grid below it is known, only a small fraction of all
 * triangles has to be tested.
 *
 * During the recursive walk down the tree, the function tracks
 * all transformations. If a leaf node is encountered, the contained
 * triangles are transformed by the tracked transformations to
 * get the absolute position of each triangle. Then all triangles
 * are sorted into the grid by their absolute position.
 *
 * \param e       Pointer to the currently processed entity
 * \param xform   Reference to the current transformation
 */
void HD_TilingTerrain::tiling_terrain(ssgEntity * e, sgMat4 xform)
{
  // only continue if HOT traversal is enabled for this entity
  if ( e->getTraversalMask() & SSGTRAV_HOT )
  {
    if ( e->isAKindOf(ssgTypeBranch()) )
    {
      ssgBranch *br = (ssgBranch *) e ;
      if ( e -> isA ( ssgTypeTransform() ) )
      {
        sgMat4 xform1;
        ((ssgTransform *)e)->getTransform ( xform1 ) ;
        sgPreMultMat4  ( xform, xform1 ) ; //Pre or Post ???
        /*
        std::cout << "------tranform " << br<< std::endl;
        std::cout << "-------------- " << xform[0][0]<<"  "<< xform[0][1]<<"  "<< xform[0][2]<<"  "<< xform[0][3]<< std::endl;
        std::cout << "-------------- " << xform[1][0]<<"  "<< xform[1][1]<<"  "<< xform[1][2]<<"  "<< xform[1][3]<< std::endl;
        std::cout << "-------------- " << xform[2][0]<<"  "<< xform[2][1]<<"  "<< xform[2][2]<<"  "<< xform[2][3]<< std::endl;
        std::cout << "-------------- " << xform[3][0]<<"  "<< xform[3][1]<<"  "<< xform[3][2]<<"  "<< xform[3][3]<< std::endl;
        */
      }
      //else std::cout << "------branch " << br<< std::endl;
      
      // Bug #16552: "xform" is actually passed by reference and
      // not by value. Therefore we have to store it locally and
      // restore it before recursing to the next child. Else all
      // children receive an xform matrix that was modified by
      // the previous child.
      sgMat4 local_xform;
      sgCopyMat4(local_xform, xform);
      for ( int i = 0 ; i < br -> getNumKids () ; i++ )
      {
        tiling_terrain ( br -> getKid ( i ), xform);
        // restore transformation matrix
        sgCopyMat4(xform, local_xform);
      }
    }
    else if ( e -> isAKindOf ( ssgTypeLeaf() ) )
    {
      //std::cout << "------leaf " << e<< std::endl;
      ssgLeaf  *leaf = (ssgLeaf  *) e ;
      int nt = leaf->getNumTriangles();
      //std::cout << "------n triangles " << nt<< std::endl;
      for ( int n = 0 ; n < nt ; n++ ) //pour chaque triangle
      {
        short iv1,iv2,iv3;/*float *v1, *v2, *v3;*/
        sgVec3 v1,v2,v3;
        leaf->getTriangle ( n, &iv1, &iv2,  &iv3 );

        sgCopyVec3 (v1 , leaf->getVertex(iv1));
        sgXformPnt3( v1, xform);
        sgCopyVec3 (v2 , leaf->getVertex(iv2));
        sgXformPnt3( v2, xform);
        sgCopyVec3 (v3 , leaf->getVertex(iv3));
        sgXformPnt3( v3, xform);
        /*
        std::cout << "------triangle " << std::endl;
        std::cout << "-------------- " << v1[0]<<"  "<< v1[1]<<"  "<< v1[2]<<"  "<< std::endl;
        std::cout << "-------------- " << v2[0]<<"  "<< v2[1]<<"  "<< v2[2]<<"  "<< std::endl;
        std::cout << "-------------- " << v3[0]<<"  "<< v3[1]<<"  "<< v3[2]<<"  "<< std::endl;
        */
        //calcule cube englobant
        sgBox box;
        box.empty();
        box.extend(v1);
        box.extend(v2);
        box.extend(v3);
        //std::cout << "Min " << box.min[0]<<", "<<box.min[1]<<", "<<box.min[2]<<" Max"<<box.max[0]<<", "<<box.max[1]<<", "<<box.max[2]<< std::endl;
        //ajoute dans cellules recouvertes
        int i1 = (int)(-0.1+box.min[0]/SIZE_CELL_GRID_PLANES) + SIZE_GRID_PLANES/2;
        int i2 = (int)(0.1+box.max[0]/SIZE_CELL_GRID_PLANES) + SIZE_GRID_PLANES/2;
        int j1 = (int)(-0.1+box.min[2]/SIZE_CELL_GRID_PLANES) + SIZE_GRID_PLANES/2;
        int j2 = (int)(0.1+box.max[2]/SIZE_CELL_GRID_PLANES) + SIZE_GRID_PLANES/2;
        if (i1 < 0)
          i1 = 0;
        if (i1 > SIZE_GRID_PLANES-1)
          i1 = SIZE_GRID_PLANES-1;
        if (i2 < 0)
          i2 = 0;
        if (i2 > SIZE_GRID_PLANES-1)
          i2 = SIZE_GRID_PLANES-1;
        if (j1 < 0)
          j1 = 0;
        if (j1 > SIZE_GRID_PLANES-1)
          j1 = SIZE_GRID_PLANES-1;
        if (j2 < 0)
          j2 = 0;
        if (j2 > SIZE_GRID_PLANES-1)
          j2 = SIZE_GRID_PLANES-1;
        //std::cout << "cellules i, j  " << i1 <<", "<<i2<<", "<<j1<<","<<j2<< std::endl;
        for (int i = i1; i <= i2; i++)
        {
          for (int j = j1; j <= j2; j++)
          {
            //std::cout << "met dans cellule " << i <<", "<<j<<  std::endl;
            tile_table[i][j]->add(v1);
            tile_table[i][j]->add(v2);
            tile_table[i][j]->add(v3);
          }
        }
      }
    }
  }
}

// utility routine
float HD_TilingTerrain::Isleft(myPoint2D p1, myPoint2D p2, myPoint2D p)
{
  return ((p2.x - p1.x) * (p.y - p1.y) - (p.x - p1.x) * (p2.y - p1.y));
}

// test si point x,y, dans la projection du trianle p1, p2,p3 sur plan horizontal
int HD_TilingTerrain::on_triangle(float x, float y, float *p1, float *p2, float *p3)
{
  int t1,t2,t3,test;
  myPoint2D a,b,c,p;
  p.x = x;//east
  p.y = y;//nord
  //p1,p2,p3 : est, up,sud
  a.x = p1[2];
  a.y = p1[0];
  b.x = p2[2];
  b.y = p2[0];
  c.x = p3[2];
  c.y = p3[0];
  t1 = (Isleft(c,a,p)>0);
  t2 = (Isleft(b,c,p)>0);
  t3 = (Isleft(a,b,p)>0);
  test =  (t1==t2) && (t1==t3);
  return test;
}
