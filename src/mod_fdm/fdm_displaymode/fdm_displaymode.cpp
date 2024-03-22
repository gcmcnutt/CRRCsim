/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2006, 2007, 2008 - Todd Templeton (original author)
 *   Copyright (C) 2008 - Jan Reucker
 *   Copyright (C) 2008 - Jens Wilhelm Wulf
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
// Created 11/09/06 Todd R. Templeton <ttemplet@eecs.berkeley.edu>
// Based on fdm_larcsim.cpp

#include "fdm_displaymode.h"

#if (MOD_FDM_USE_DISPLAYMODE != 0)

# include <math.h>

#include "../../mod_misc/ls_constants.h"
#include "../ls_geodesy.h"

#define SERVO_PACKET_SIZE 60

#ifdef MIN
#undef MIN
#endif
#define MIN(a,b) (a < b ? a : b)
#ifdef MAX
#undef MAX
#endif
#define MAX(a,b) (a > b ? a : b)


void CRRC_AirplaneSim_DisplayMode::update(TSimInputs* inputs,
                                          double      dt,
                                          int         multiloop)
{
  int retval;
  unsigned short servos[8];
  char servobuf[SERVO_PACKET_SIZE] = {0,};
  int nBytesServo = 0;
  unsigned long sum = 0;
  int i, j;

  while (1)
  {
    retval = io->read(&buf[nBytes], DATA_PACKET_SIZE - nBytes);
    if (retval > 0)
      nBytes += retval;
    else
      break;
    if (nBytes >= DATA_PACKET_SIZE)
    {
      decodePacket(buf);
      nBytes = 0;
    }
  }
  
  servos[0] = (unsigned short)(65536.0 * MAX(-0.5, MIN(inputs->aileron,  0.5)) + 32768.0);
  servos[1] = (unsigned short)(65536.0 * MAX(-0.5, MIN(inputs->elevator, 0.5)) + 32768.0);
  servos[2] = (unsigned short)(65536.0 * MAX(-0.5, MIN(inputs->rudder,   0.5)) + 32768.0);
  servos[3] = (unsigned short)(65536.0 * MAX( 0.0, MIN(inputs->throttle, 1.0)));
  for (i = 4, j = 0; i < 8 && j < TSimInputs::NUM_AUX_INPUTS; i++, j++)
    servos[i] = (unsigned short)(65536.0 * MAX(-0.5, MIN(inputs->aux[j], 0.5)) + 32768.0);
  for (; i < 8; i++)
    servos[i] = 0;
  for (i = 0; i < 8; i++)
    nBytesServo += sprintf(&servobuf[nBytesServo], "%6hx ", servos[i]);
  nBytesServo += sprintf(&servobuf[nBytesServo], " end");
  
  for (i = 0; i < SERVO_PACKET_SIZE - 1; i++)
    sum += servobuf[i];
  servobuf[SERVO_PACKET_SIZE - 1] = (char)(sum % 256);
  
  io->write(servobuf, SERVO_PACKET_SIZE);
}

void CRRC_AirplaneSim_DisplayMode::decodePacket(char* buf)
{
  double phi, the, psi;
  double lat, lon, alt;
  int i, sum = 0;
  char tmp[20],tmpr[20];

  for(i=0;i<199;i++) sum += buf[i];
  //if((buf[199] & 0x0000ff) == sum%256) {
    
    sscanf(buf,"%lf %lf %lf %s %s %lf",
           &phi,&the,&psi,
           tmp        , tmpr    , &alt);
  lat = atof(tmp);
  lon = atof(tmpr);

  // fix origin (for when display is used for real aircraft with real lat/lon,
  //   instead of using simulated data that is always near (lat,lon) = (0,0))
  if (!originInitialized)
  {
    origin[0] = lat;
    origin[1] = lon;
    originInitialized = true;
  }
  lat -= origin[0];
  lon -= origin[1];
        
  euler_angles_v[0] = phi;
  euler_angles_v[1] = the;
  euler_angles_v[2] = psi;
  
  Latitude  = lat * M_PI / 180.0;
  Longitude = lon * M_PI / 180.0;
  Altitude  = alt * M_TO_FT;
  
  ls_geod_to_geoc( Latitude, Altitude, &Sea_level_radius, &Lat_geocentric);
  v_P_CG_Rwy.r[0] = Sea_level_radius * Latitude;
  v_P_CG_Rwy.r[1] = Sea_level_radius * Longitude;
  v_P_CG_Rwy.r[2] = -Altitude;
  
  v_V_local_rel_ground.r[0] = 0.0;
  v_V_local_rel_ground.r[1] = 0.0;
  v_V_local_rel_ground.r[2] = 0.0;
  
  v_V_dot_local.r[0] = 0.0;
  v_V_dot_local.r[1] = 0.0;
  v_V_dot_local.r[2] = 0.0;
  
  v_R_omega_body.r[0] = 0.0;
  v_R_omega_body.r[1] = 0.0;
  v_R_omega_body.r[2] = 0.0;
}



bool CRRC_AirplaneSim_DisplayMode::UseMe(SimpleXMLTransfer* cfg)
{
  return(
         cfg->getInt("simulation.display_mode.fUse", 0) != 0
         );
}

void CRRC_AirplaneSim_DisplayMode::LoadFromXML(SimpleXMLTransfer* xml,
                                               SimpleXMLTransfer* cfg)
{
  if (UseMe(cfg))
  {
    io = new CharDeviceWrapper(cfg->getString("simulation.display_mode.device", "udpserver,127.0.0.1/0.0.0.0,9003").c_str(), false);
    io->set_max_read_interval(3.0);
    nBytes = 0;
    originInitialized = false;
  }
  else
  {
    // actually, I am not configured! So just throw an exception...
    throw XMLException("display_mode is not set");
  }
}

CRRC_AirplaneSim_DisplayMode::CRRC_AirplaneSim_DisplayMode(const char* filename,
                                                           SimpleXMLTransfer* cfg) : CRRC_AirplaneSim_Larcsim(filename, 0, cfg)
{
  SimpleXMLTransfer* fileinmemory = new SimpleXMLTransfer(filename);
  
  LoadFromXML(fileinmemory, cfg);
  
  delete fileinmemory;
}

CRRC_AirplaneSim_DisplayMode::CRRC_AirplaneSim_DisplayMode(SimpleXMLTransfer* xml,
                                                           SimpleXMLTransfer* cfg) : CRRC_AirplaneSim_Larcsim(xml, 0, cfg)
{
  LoadFromXML(xml, cfg);
}

CRRC_AirplaneSim_DisplayMode::~CRRC_AirplaneSim_DisplayMode()
{
  delete io;
}
#endif
