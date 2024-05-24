/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2006, 2008 - Todd Templeton (original author)
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
// Based on tx_serial2.cpp

#include "../../crrc_main.h"
#include "../../global.h"
#include "../../aircraft.h"
#include "../../SimStateHandler.h"
#include "../../mod_fdm/fdm.h"
#include "inputdev_autoc.h"

#include <stdio.h>

#define PI 3.141592653589793
#define FEET2METERS 0.3048


T_TX_InterfaceAUTOC::T_TX_InterfaceAUTOC()
{
#if DEBUG_TX_INTERFACE > 0
  printf("T_TX_InterfaceAUTOC::T_TX_InterfaceAUTOC()\n");
#endif  
  input = (AUTOC*)0;
}

T_TX_InterfaceAUTOC::~T_TX_InterfaceAUTOC()
{
#if DEBUG_TX_INTERFACE > 0
  printf("T_TX_InterfaceAUTOC::~T_TX_InterfaceAUTOC()\n");
#endif  
  if (input != (AUTOC*)0)
    delete input;
}

int T_TX_InterfaceAUTOC::init(SimpleXMLTransfer* config)
{
#if DEBUG_TX_INTERFACE > 0
  printf("int T_TX_InterfaceAUTOC::init(SimpleXMLTransfer* config)\n");
#endif  
  char devicestr[100];

  T_TX_Interface::init(config);
  
  device   = config->getString("inputMethod.autoc.device", "udpserver,127.0.0.1/0.0.0.0,9002");
  strncpy(devicestr, device.c_str(), 100); devicestr[99] = '\0';
  input = new AUTOC(devicestr);
  cnt_cmd[0] = 0;
  cnt_cmd[1] = 0;
  cnt_cmd[2] = 0;
  reverse = 0;
  
  return(0);
}

void T_TX_InterfaceAUTOC::putBackIntoCfg(SimpleXMLTransfer* config)
{
#if DEBUG_TX_INTERFACE > 0
  printf("int T_TX_InterfaceAUTOC::putBackIntoCfg(SimpleXMLTransfer* config)\n");
#endif  
  T_TX_Interface::putBackIntoCfg(config);
  
  config->setAttributeOverwrite("inputMethod.autoc.device",   device);  
}

void T_TX_InterfaceAUTOC::getInputData(TSimInputs* inputs)
{
#if DEBUG_TX_INTERFACE > 1
  printf("void T_TX_InterfaceAUTOC::getInputData(TSimInputs* inputs)\n");
#endif  

  struct imu imudata;
  struct gps gpsdata;
  struct servo servopacket;
  
  if ((!Global::testmode) && (Global::aircraft->getFDM() != NULL))
  {
    double phi, the, psi;
    double cphi, sphi, cthe, sthe, cpsi, spsi;
    double r11, r12, r13, r21, r22, r23, r31, r32, r33;
    CRRCMath::Vector3 vel, waccel, accel, pqr;
    unsigned long current_time = Global::Simulation->getSimulationTimeSinceReset();

    phi    = Global::aircraft->getFDM()->getPhi();
    the    = Global::aircraft->getFDM()->getTheta();
    psi    = Global::aircraft->getFDM()->getPsi();
    vel    = Global::aircraft->getFDM()->getVel();
    waccel = Global::aircraft->getFDM()->getAccel();
    waccel.r[2] += 0.03 / FEET2METERS; // correct for bias
    waccel.r[2] -= 9.80665 / FEET2METERS; // include acceleration due to gravity
    pqr    = Global::aircraft->getFDM()->getPQR();

    // fix orientations by multiples of 2*PI
    phi += (phi > 0.0 ? -1.0 : 1.0) * floor(fabs(phi) / (2*PI)) * 2*PI;
    if(phi > PI)
      phi -= 2*PI;
    if(phi < -PI)
      phi += 2*PI;
    the += (the > 0.0 ? -1.0 : 1.0) * floor(fabs(the) / (2*PI)) * 2*PI;
    if(the > PI)
      the -= 2*PI;
    if(the < -PI)
      the += 2*PI;
    psi += (psi > 0.0 ? -1.0 : 1.0) * floor(fabs(psi) / (2*PI)) * 2*PI;
    if(psi > PI)
      psi -= 2*PI;
    if(psi < -PI)
      psi += 2*PI;

    // put accelerations into body frame
    cphi = cos(phi);
    sphi = sin(phi);
    cthe = cos(the);
    sthe = sin(the);
    cpsi = cos(psi);
    spsi = sin(psi);
    r11 = cpsi * cthe;
    r12 = cpsi * sthe * sphi - spsi * cphi;
    r13 = cpsi * sthe * cphi + spsi * sphi;
    r21 = spsi * cthe;
    r22 = spsi * sthe * sphi + cpsi * cphi;
    r23 = spsi * sthe * cphi - cpsi * sphi;
    r31 = -sthe;
    r32 = cthe * sphi;
    r33 = cthe * cphi;
    accel.r[0] = r11 * waccel.r[0] + r21 * waccel.r[1] + r31 * waccel.r[2];
    accel.r[1] = r12 * waccel.r[0] + r22 * waccel.r[1] + r32 * waccel.r[2];
    accel.r[2] = r13 * waccel.r[0] + r23 * waccel.r[1] + r33 * waccel.r[2];
    //NOTE: looks like angular rates are already in body frame
  
    imudata.p        = pqr.r[0]; // angular velocities (radians/sec)
    imudata.q        = pqr.r[1];
    imudata.r        = pqr.r[2];
    imudata.ax       = accel.r[0] * FEET2METERS; // acceleration (m/s^2)
    imudata.ay       = accel.r[1] * FEET2METERS;
    imudata.az       = accel.r[2] * FEET2METERS;
    imudata.hx       = -r11 / 2.0; // magnetic field
    imudata.hy       = -r12 / 2.0; //NOTE: all of these negated because AUTOC magnetic sensor is negated
    imudata.hz       = -r13 / 2.0;
    imudata.Ps       = Global::aircraft->getFDM()->getAlt() * FEET2METERS; // static pressure (altitude in m)
    imudata.Pt       = sqrt(vel.r[0]*vel.r[0] + vel.r[1]*vel.r[1] + vel.r[2]*vel.r[2]) * FEET2METERS; // pitot pressure (m/s): sent and displayed, but not used
    imudata.Tx       = 0; // temperature (sent but not used)
    imudata.Ty       = 0;
    imudata.Tz       = 0;
    imudata.phi      = phi; // attitudes (radians) (not sent)
    imudata.the      = the;
    imudata.psi      = psi;
    imudata.err_type = 0; // not sent
    imudata.time     = (double)current_time * 0.001;
    
    gpsdata.lat      = Global::aircraft->getFDM()->getLat() * 180.0 / PI; // degrees
    //gpsdata.lat     += 42.4159; //FIXME: location really should include proper lat/lon
    gpsdata.lon      = Global::aircraft->getFDM()->getLon() * 180.0 / PI; // degrees
    //gpsdata.lon     += -71.3980; //FIXME: location really should include proper lat/lon
    gpsdata.alt      = Global::aircraft->getFDM()->getAlt() * FEET2METERS; // m
    gpsdata.ve       = vel.r[1] * FEET2METERS; // m/s
    gpsdata.vn       = vel.r[0] * FEET2METERS;
    gpsdata.vd       = vel.r[2] * FEET2METERS;
    gpsdata.ITOW     = (uint16_t)current_time;
    gpsdata.err_type = 0; // not sent
    gpsdata.time     = (double)current_time * 0.001;
    //printf("lon=%.6lf lat=%.6lf\n", gpsdata.lon, gpsdata.lat);
    
    servopacket.chn[0] = 0x8000;
    servopacket.chn[1] = 0x8000;
    servopacket.chn[2] = 0xe000;
    servopacket.chn[3] = 0; // unused
    servopacket.chn[4] = 1000; // whether autopilot is enabled (<= 12000 for enabled, > 12000 && < 60000 for disabled)
    servopacket.chn[5] = 0; // unused
    servopacket.chn[6] = 0; // unused
    servopacket.chn[7] = 0; // unused
    servopacket.status = reverse;

    // Display data
    //input->display_message(&imudata, &gpsdata);
    
    // Send data
    input->put_state_data(&imudata, &gpsdata, &servopacket);
  }
  
  // Read data
  if(input->get_servo_cmd(cnt_cmd, &reverse) > 0)
  {
    float cnt_cmd_cnv[3];

    //fprintf(stderr, "Got servo commands...\n");
    cnt_cmd_cnv[0] = ((float)((reverse & (uint8_t)0x01) ? 65536 - cnt_cmd[0] : cnt_cmd[0]) - 32768.0) / 65536.0; //FIXME: for some reason, AUTOC sensor code uses (22418 - cnt_cmd) instead of (65536 - cnt_cmd)
    cnt_cmd_cnv[1] = ((float)((reverse & (uint8_t)0x02) ? 65536 - cnt_cmd[1] : cnt_cmd[1]) - 32768.0) / 65536.0; //FIXME: for some reason, AUTOC sensor code uses (22418 - cnt_cmd) instead of (65536 - cnt_cmd)
    cnt_cmd_cnv[2] = (float)((reverse & (uint8_t)0x04) ? 65536 - cnt_cmd[2] : cnt_cmd[2]) / 65536.0; //FIXME: for some reason, AUTOC sensor code uses (22418 - cnt_cmd) instead of (65536 - cnt_cmd)
    inputs->elevator = -(cnt_cmd_cnv[1] - cnt_cmd_cnv[0]) / 2.0;
    //inputs->rudder   =
    inputs->aileron  = (cnt_cmd_cnv[1] + cnt_cmd_cnv[0]) / 2.0;
    inputs->throttle = cnt_cmd_cnv[2];

    //fprintf(stderr, "[servo]: 0:0x%04hx 1:0x%04hx 2:0x%04hx reverse:0x%04hx\n\n", cnt_cmd[0], cnt_cmd[1], cnt_cmd[2], (uint16_t)reverse);
  }
}
