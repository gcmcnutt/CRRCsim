/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 *  Copyright (C) 2010 Joel Lienard (original author)
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
  

/*************
 * crrc_wind_from_terrain.cpp
 * 
 * very simple wind calculation from terrain profil
 *
 *********/
 
 /*
 We make a modelling locally the ground by a simple shape.
 That modelling is made from some points (2 - 10) in the vertical plane aligned with the wind.
 The distance from these measurement points of the terrain is all the bigger as the height is big.
 Mode 1: modelling by a plane
 Mode 2: modelling by in-plane potential flow with 2D panel method
 */

#include <crrc_config.h>
 
#include <plib/ssg.h>

#include "../global.h"
#include "../crrc_main.h"
#include "model_based_scenery.h"
#include "wind_from_terrain.h"


#define H_MIN     1.            // minimum height above terrain for wind estimation (1 foot)
 
/*
data used by simple 2D panel method
*/
#define N_UP_PTS  6             // number of up/down-stream points used
#define NPTS      (2*N_UP_PTS)  // total number of points defining terrain profile (panels)
#define NPAN      (NPTS-1)      // total number of panels
#define REF_L     300.          // reference length to smooth terrain slope beyond land's end
#define REF_Z     100.          // reference height from terrain
#define POW_Z     0.5           // power law to scale ratio of actual to reference height
#define BASELEN   0.5           // first panel length realtive to height from terrain
#define RATE      1.5           // panel length growth rate
#define EPS_END   .1            // accuracy in defining streamwise position of land's end
#define IT_MAX    20            // max iter in defining streamwise position of land's end
#define DELTAX    10.           // to compute terain slope at land's end

static float x[NPTS], z[NPTS];    // panel vertex coords
static float cx[NPAN], cz[NPAN];  // panel control point coords
static float cc[NPAN], ss[NPAN];  // panel cosine & sine
static float A[NPAN][NPAN];       // system matrix
static float src[NPAN];           // unknown sources

/**
 * Compute normal velocity induced on control point of panel i
 * by a uniform source strength distribution on panel j
 *
 */
static float vn_src0(int i, int j)
{
	float xr1, zr1, xr2, zr2, rq1, rq2, b0, c0, d0, e0;

  if (i == j)
  {
    return M_PI;
  }
  else
  {
    xr1 = cx[i] - x[j];
    zr1 = cz[i] - z[j];
    xr2 = cx[i] - x[j+1];
    zr2 = cz[i] - z[j+1];
    rq1 = xr1*xr1 + zr1*zr1;
    rq2 = xr2*xr2 + zr2*zr2;
    b0 = atan2(zr2*xr1 - xr2*zr1, xr2*xr1 + zr2*zr1);
    c0 = ss[i]*cc[j] - cc[i]*ss[j];
    d0 = cc[i]*cc[j] + ss[i]*ss[j];
    e0 = .5*log(rq2/rq1);
    return c0*e0 + d0*b0;
  }
}

/**
 * Compute velocity components vx,vz induced on point px,pz
 * by a uniform source strength distribution on panel j.
 *
 */
static void v_src0(float px, float pz, int j, float *vx, float *vz)
{
	float xr1, zr1, xr2, zr2, rq1, rq2, b0, c0, d0, e0;

  xr1 = px - x[j];
  zr1 = pz - z[j];
  xr2 = px - x[j+1];
  zr2 = pz - z[j+1];
  rq1 = xr1*xr1 + zr1*zr1;
  rq2 = xr2*xr2 + zr2*zr2;
  b0 = atan2(zr2*xr1 - xr2*zr1, xr2*xr1 + zr2*zr1);
  c0 = -ss[j];
  d0 = cc[j];
  e0 = .5*log(rq2/rq1);
  *vz = c0 * e0 + d0 * b0;
  *vx = c0 * b0 - d0 * e0;
}

/**
 * Solve linear system by Gauss elimination
 *
 */
static void solve_gs(float A[][NPAN], float x[], int n)
{
  for(int k = 0; k < n-1; k++)
    for(int i = n-1; i > k; i--)
    {
      A[i][k] /= A[k][k];
      for(int j = n-1; j > k; j--)
        A[i][j] -= A[k][j]*A[i][k];
    }
 
  for(int i = 1; i < n; i++)
    for(int j = 0; j < i; j++)
      x[i] -= A[i][j]*x[j];
  x[n-1] /= A[n-1][n-1];
  for(int i = n-2; i >= 0; i--)
  {
    for(int j = n-2; j > i; j--)
      x[i] -= A[i][j]*x[j];
    x[i] /= A[i][i];
  }
}

int wind_from_terrain(double X, double Y, double Z,
                      float *x_wind_velocity, float *y_wind_velocity, float *z_wind_velocity)
{
  // freestream wind velocity
  float flWindVel = cfg->wind->getVelocity();
  float flWindDir = cfg->wind->getDirection()*M_PI/180.;
  float flWindDirX = cos(flWindDir); //upstream versor
  float flWindDirY = sin(flWindDir); //upstream versor
  
  float z_c = Global::scenery->getHeight(X, Y); //terrain height below the point
  float H = -Z; //positive down -> positive up
  if ((H - z_c) < H_MIN)
    H = z_c + H_MIN;
  float dz = H - z_c;
  
  // if is above a defined terrain, estimate wind speed from terrain
  if (z_c > DEEPEST_HELL)
  {
    sgVec3 wind;
    sgSetVec3(wind, 0, 0, 0);

    if (Global::wind_mode == 1)
    {
      //
      //We tilt the vector of wind along the slope, with the same speed in module
      //
      const float COEF = 0.5; //define angle of measure
      float dx = COEF*dz*flWindDirX; //upstream vector
      float dy = COEF*dz*flWindDirY; //upstream vector
      
      float z_f = Global::scenery->getHeight(X+dx, Y+dy);//terrain height upstream
      if (z_f==DEEPEST_HELL) { z_f = z_c;}
      float z_b = Global::scenery->getHeight(X-dx, Y-dy);//terrain height downstream
      if (z_b==DEEPEST_HELL) { z_b = z_c;}
      sgVec3 p_c, p_f, p_b;
      sgSetVec3(p_c, X, Y, -z_c);
      sgSetVec3(p_f, X+dx, Y+dy, -z_f);
      sgSetVec3(p_b, X-dx, Y-dy, -z_b);
      //float z_l = Global::scenery->getHeight(X+dx, Y-dsin);//left
      //float z_r = Global::scenery->getHeight(X-dx, Y+dy);//right
      //sgVec3 p_0, p_l, p_r;
      //sgSetVec3(p_0,X,Y,H);

      sgVec3 dir;
      sgSubVec3(dir, p_f, p_b);
      sgNormaliseVec3(dir); //-> Unit vector in the direction of the wind
      sgScaleVec3(wind, dir, -flWindVel); 
    }
    else if (Global::wind_mode == 2)
    {
      //
      //2D potential flow in a wind-aligned vertical plane
      //
      
      // define panels vertex points in a reference system with x axis aligned 
      // with wind direction (negative upstream) and origin on the point X,Y   
      if (dz < 0.1*REF_Z) dz = 0.1*REF_Z; // do not reduce ref length too much
      float dd = 0.5*BASELEN*pow(dz/REF_Z,POW_Z)*REF_Z;
      float ds = dd;
      float d1 = 0.0;
      float d2 = 0.0;
      float dzdd1 = 0.0;
      float dzdd2 = 0.0;
      float z1 = DEEPEST_HELL;
      float z2 = DEEPEST_HELL;
      for(int i=1; i <= N_UP_PTS; i++)
      {
        float dx = ds*flWindDirX;
        float dy = ds*flWindDirY;
        
        x[N_UP_PTS-i]   = -ds;
        z[N_UP_PTS-i]   = Global::scenery->getHeight(X+dx, Y+dy);
        x[N_UP_PTS-1+i] = +ds;
        z[N_UP_PTS-1+i] = Global::scenery->getHeight(X-dx, Y-dy);

        // check if upwind point is outside defined terrain profile
        // in case find terrain elevation and slope at upstream land's end
        if (z[N_UP_PTS-i] == DEEPEST_HELL)
        {
          if (z1 == DEEPEST_HELL)
          {
            float xa = x[N_UP_PTS-i+1];
            float za = z[N_UP_PTS-i+1];
            float xb = x[N_UP_PTS-i];
            float xc, zc;
            int it = 0;
            while ((fabs(xa - xb) > EPS_END) && it < IT_MAX)
            {
              it++;
              xc = 0.5*(xa + xb);
              zc = Global::scenery->getHeight(X-xc*flWindDirX, Y-xc*flWindDirY);
              if (zc == DEEPEST_HELL)
                xb = xc;
              else
              {
                xa = xc;
                za = zc;
              }
            }
            d1 = xa;
            z1 = za;
            // estimate terrain slope at land's end
            xc = xa + DELTAX;
            zc = Global::scenery->getHeight(X-xc*flWindDirX, Y-xc*flWindDirY);
            dzdd1 = (z1 - zc)/DELTAX;
          }
          z[N_UP_PTS-i] = z1 + dzdd1*REF_L*(1. - exp(-fabs(x[N_UP_PTS-i] - d1)/REF_L));
        }
        // check if downwind point is outside defined terrain profile
        // in case find terrain elevation and slope at downstream land's end
        if (z[N_UP_PTS-1+i] == DEEPEST_HELL)
        {
          if (z2 == DEEPEST_HELL)
          {
            float xa = x[N_UP_PTS-1+i-1];
            float za = z[N_UP_PTS-1+i-1];
            float xb = x[N_UP_PTS-1+i];
            float xc, zc;
            int it = 0;
            while ((fabs(xa - xb) > EPS_END) && it < IT_MAX)
            {
              it++;
              xc = 0.5*(xa + xb);
              zc = Global::scenery->getHeight(X-xc*flWindDirX, Y-xc*flWindDirY);
              if (zc == DEEPEST_HELL)
                xb = xc;
              else
              {
                xa = xc;
                za = zc;
              }
            }
            d2 = xa;
            z2 = za;
            // estimate terrain slope at land's end
            xc = xa - DELTAX;
            zc = Global::scenery->getHeight(X-xc*flWindDirX, Y-xc*flWindDirY);
            dzdd2 = (z2 - zc)/DELTAX;
          }
          z[N_UP_PTS-1+i] = z2 + dzdd2*REF_L*(1. - exp(-fabs(x[N_UP_PTS-1+i] - d2)/REF_L));
        }
        
        dd *= (i == 1 ? 2. : 1.)*RATE;
        ds += dd;
      }
      
      // compute panels geometry
      for(int i = 0; i < NPAN; i++)
      {
        cx[i] = .5*(x[i+1] + x[i]);
        cz[i] = .5*(z[i+1] + z[i]);
        float lx = x[i+1] - x[i];
        float lz = z[i+1] - z[i];
        float ll = hypot(lx, lz);
        ss[i] = lz/ll;
        cc[i] = lx/ll;
      }
      
      // construct system matrix and source term
      for(int i = 0; i < NPAN; i++)
      {
        src[i] = ss[i]; // freestream flow = horizontal wind
        for( int j = 0; j < NPAN; j++ )
          A[i][j] = vn_src0(i, j);
      }
      
      // solve system matrix for the unknown source/vortex strength
      solve_gs(A, src, NPAN);

      // compute velocity induced on target point
      float vx = 1.; // freestream flow = horizontal wind
      float vz = 0.; // freestream flow = horizontal wind
      for(int i = 0; i < NPAN; i++)
      {
        float dvx, dvz;
        v_src0(0., H, i, &dvx, &dvz); // induced (in plane) velocity on target point
        vx += src[i]*dvx;
        vz += src[i]*dvz;
      }

      // define resulting wind vector
      sgSetVec3(wind, flWindDirX*vx, flWindDirY*vx, vz);
      sgScaleVec3(wind, -flWindVel); 
    }
      
    // return results
    *x_wind_velocity = wind[0];
    *y_wind_velocity = wind[1];
    *z_wind_velocity = wind[2];
    
    return 0;
  }
  else
  {
    // point is invalid
    return 1;
  }
}
