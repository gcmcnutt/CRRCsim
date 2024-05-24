/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2006, 2008 - Todd Templeton (original author)
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
// Based on imugps.c and globaldefs.h in MNAV autopilot, http://sourceforge.net/projects/micronav
// Header from original imugps.c in MNAV autopilot:

/******************************************************************************
* FILE: imugps.c
* DESCRIPTION:
*
*
*
* SOURCE:
* REVISED: 4/05/06 Jung Soon Jang
******************************************************************************/

// Header from original globaldefs.h in MNAV autopilot:

/******************************************************************************
 *global defintions used in the avionics program
 ******************************************************************************/

// License of original MNAV autopilot: GPL v2
//   (from COPYING file in MNAV distribution)
// Contact info for original MNAV author: Jung Soon Jang <jjang@xbow.com>
//   (from release.txt in MNAV distribution)

#ifndef __AUTOC_H__
#define __AUTOC_H__

#include "../../mod_chardevice/chardevice.h"

// borrow definitions of imu/gps/etc...
#include "../inputdev_mnav/mnav.h"

class AUTOC : BufferedCharDevice
{
protected:
  uint16_t littleendians(uint16_t a);
  uint16_t bigendians(uint16_t a);
  uint32_t littleendianl(uint32_t a);
  uint32_t bigendianl(uint32_t a);
  
  bool checksum(uint8_t* buffer, int packet_len);
  
  void encode_gpspacket(struct gps *data, uint8_t* buffer);
  void encode_ahrspacket(struct imu *data, uint8_t* buffer);
  void encode_servopacket(struct servo *servopacket, uint8_t* buffer);
  void encode_packet(struct imu *data, struct gps *gpsdata, struct servo *servopacket, uint8_t* buffer);
  
  void decode_servo_cmd(uint8_t data[24], uint16_t cnt_cmd[3]);
  
public:
  void put_state_data(struct imu *imudata, struct gps *gpsdata, struct servo *servopacket);
  int get_servo_cmd(uint16_t cnt_cmd[3], uint8_t *reverse);
  void display_message(struct imu *data, struct gps *gdata);

  void process_input(void)
  {
  }
  
  AUTOC(char* device)
  {
    init(device, false);
    charDevice->set_max_read_interval(3.0);
  }

  ~AUTOC()
  {
    cleanup();
  }
};

#endif //__AUTOC_H__
