/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2006, 2007, 2008 - Todd Templeton (original author)
 *   Copyright (C) 2007 - Jan Reucker
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
// Based on imugps.c in MNAV autopilot, http://sourceforge.net/projects/micronav
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

// License of original MNAV autopilot: GPL v2
//   (from COPYING file in MNAV distribution)
// Contact info for original MNAV author: Jung Soon Jang <jjang@xbow.com>
//   (from release.txt in MNAV distribution)

#include <stdlib.h>
#include <stdio.h>
#if 0
/* this does not work for *BSD, include <sys/types.h> instead (below) */
#if defined(__APPLE__) || defined(MACOSX)
#include <machine/endian.h>
#elif !defined(WIN32)
#include <endian.h>
#endif
#else
#ifndef WIN32
/* includes endian.h from wherever it might be */
#include <sys/types.h>
#endif
#endif
#include "mnav.h"

//#define OUTPUT_PACKET_TYPE 'S'
//#define OUTPUT_PACKET_SIZE 51

#define OUTPUT_PACKET_TYPE 'N'
#define OUTPUT_PACKET_SIZE 86

//#define OUTPUT_PACKET_TYPE 'I'
//#define OUTPUT_PACKET_SIZE 93

#define SERVO_PACKET_SIZE 24


inline uint16_t MNAV::littleendians(uint16_t a)
{
  #if __BYTE_ORDER == __LITTLE_ENDIAN
  return a;
  #else
  uint8_t* b = (uint8_t*)&a;
  uint8_t tmp;
  tmp = b[0]; b[0] = b[1]; b[1] = tmp;
  return a;
  #endif
}

inline uint16_t MNAV::bigendians(uint16_t a)
{
  #if __BYTE_ORDER == __LITTLE_ENDIAN
  uint8_t* b = (uint8_t*)&a;
  uint8_t tmp;
  tmp = b[0]; b[0] = b[1]; b[1] = tmp;
  return a;
  #else
  return a;
  #endif
}

inline uint32_t MNAV::littleendianl(uint32_t a)
{
  #if __BYTE_ORDER == __LITTLE_ENDIAN
  return a;
  #else
  uint8_t* b = (uint8_t*)&a;
  uint8_t tmp;
  tmp = b[0]; b[0] = b[3]; b[3] = tmp;
  tmp = b[1]; b[1] = b[2]; b[2] = tmp;
  return a;
  #endif
}

inline uint32_t MNAV::bigendianl(uint32_t a)
{
  #if __BYTE_ORDER == __LITTLE_ENDIAN
  uint8_t* b = (uint8_t*)&a;
  uint8_t tmp;
  tmp = b[0]; b[0] = b[3]; b[3] = tmp;
  tmp = b[1]; b[1] = b[2]; b[2] = tmp;
  return a;
  #else
  return a;
  #endif
}


void MNAV::put_state_data(struct imu *imudata, struct gps *gpsdata, struct servo *servopacket)
{
  uint8_t buffer[OUTPUT_PACKET_SIZE];
  
  //encode_gpspacket(gpsdata, &buffer[31]);
  encode_packet(imudata, gpsdata, servopacket, buffer);

  //sendout the command packet
  //while (nbytes != OUTPUT_PACKET_SIZE) nbytes = write(sPort0,(char*)buffer, OUTPUT_PACKET_SIZE);
  charDevice->write((const void*)buffer, OUTPUT_PACKET_SIZE);
}


/***************************************************************************************
 *check the checksum of the data packet
 ***************************************************************************************/
bool MNAV::checksum(uint8_t* buffer, int packet_len)
{
  uint16_t          i = 0, rcvchecksum = 0;
  unsigned long sum = 0;

  for(i = 2; i < packet_len - 2; i++)
    sum = sum + buffer[i];
  //rcvchecksum = ((rcvchecksum = buffer[packet_len-2]) << 8) | buffer[packet_len-1];
  rcvchecksum  = buffer[packet_len-2];
  rcvchecksum  = rcvchecksum << 8;
  rcvchecksum |= buffer[packet_len-1];
 
  if (rcvchecksum == (sum&0xFFFF))
    return true;
  else
    return false;
}


/***************************************************************************************
 *encode the gps data packet
 ***************************************************************************************/
void MNAV::encode_gpspacket(struct gps *data, uint8_t* buffer)
{
   //signed long tmp=0;
   int tmp32;
   short i;
   uint16_t  sum=0;
   
   buffer[ 0] = 0x55;
   buffer[ 1] = 0x55;
   buffer[ 2] = 'G';

   /* gps velocity in m/s */ 
   //data->vn =(double)((((((tmp = (signed char)buffer[ 6]<<8)|buffer[ 5])<<8)|buffer[ 4])<<8)|buffer[ 3])*1.0e-2; tmp=0;
   tmp32=(int)(data->vn *1.0e2); *((uint32_t*)&buffer[ 3])=littleendianl(*((uint32_t*)&tmp32));
   //data->ve =(double)((((((tmp = (signed char)buffer[10]<<8)|buffer[ 9])<<8)|buffer[ 8])<<8)|buffer[ 7])*1.0e-2; tmp=0;
   tmp32=(int)(data->ve *1.0e2); *((uint32_t*)&buffer[ 7])=littleendianl(*((uint32_t*)&tmp32));
   //data->vd =(double)((((((tmp = (signed char)buffer[14]<<8)|buffer[13])<<8)|buffer[12])<<8)|buffer[11])*1.0e-2; tmp=0;
   tmp32=(int)(data->vd *1.0e2); *((uint32_t*)&buffer[11])=littleendianl(*((uint32_t*)&tmp32));

   /* gps position */
   //data->lon=(double)((((((tmp = (signed char)buffer[18]<<8)|buffer[17])<<8)|buffer[16])<<8)|buffer[15]);  	 tmp=0;
   tmp32=(int)(data->lon*1.0e7); *((uint32_t*)&buffer[15])=littleendianl(*((uint32_t*)&tmp32));
   //data->lat=(double)((((((tmp = (signed char)buffer[22]<<8)|buffer[21])<<8)|buffer[20])<<8)|buffer[19]);  	 tmp=0;
   tmp32=(int)(data->lat*1.0e7); *((uint32_t*)&buffer[19])=littleendianl(*((uint32_t*)&tmp32));
   //data->alt=(double)((((((tmp = (signed char)buffer[26]<<8)|buffer[25])<<8)|buffer[24])<<8)|buffer[23])*1.0e-3; tmp=0;
   tmp32=(int)(data->alt*1.0e3); *((uint32_t*)&buffer[23])=littleendianl(*((uint32_t*)&tmp32));
   //data->lon=(data->lon)*1.0e-7;
   //data->lat=(data->lat)*1.0e-7;
   

   /* gps time */
   //data->ITOW = ((data->ITOW = buffer[28]) << 8)|buffer[27];
   *((uint32_t*)&buffer[27]) = littleendians(*((uint16_t*)&data->ITOW));
   //data->err_type = TRUE;
   //data->time = get_Time();
   
   buffer[29] = 0;
   buffer[30] = 0;
   buffer[31] = 0;
   buffer[32] = 0;
   
   for(i=2;i<33;i++) sum += buffer[i];
   
   buffer[33] = (uint8_t)(sum >> 8);
   buffer[34] = (uint8_t)sum;
}

void MNAV::encode_ahrspacket(struct imu *data, uint8_t* buffer)
{
   signed short tmp16;
   
   buffer[ 0] = 'A';

   /* angle in rad */
   //data->phi  = (double)(((tmp = (signed char)buffer[ 9])<<8)|buffer[10])*1.065264436e-04; tmp=0;
   tmp16 = (signed short)(data->phi*0.9387340515702713e04); *((uint16_t*)&buffer[ 1]) = bigendians(*((uint16_t*)&tmp16));
   //data->the  = (double)(((tmp = (signed char)buffer[11])<<8)|buffer[12])*1.065264436e-04; tmp=0;
   tmp16 = (signed short)(data->the*0.9387340515702713e04); *((uint16_t*)&buffer[ 3]) = bigendians(*((uint16_t*)&tmp16));
   //data->psi  = (double)(((tmp = (signed char)buffer[13])<<8)|buffer[14])*1.065264436e-04; tmp=0;
   tmp16 = (signed short)(data->psi*0.9387340515702713e04); *((uint16_t*)&buffer[ 5]) = bigendians(*((uint16_t*)&tmp16));
}

void MNAV::encode_servopacket(struct servo *servopacket, uint8_t* buffer)
{
   buffer[ 0] = 'F';
   //servopacket.status = buffer[ 0];
   buffer[ 1] = servopacket->status;
   //servopacket.chn[0] = ((tmpr = buffer[33]) << 8)|buffer[34]; tmpr = 0;
   *((uint16_t*)&buffer[ 2]) = bigendians(*((uint16_t*)&servopacket->chn[0]));
   //servopacket.chn[1] = ((tmpr = buffer[35]) << 8)|buffer[36]; tmpr = 0;
   *((uint16_t*)&buffer[ 4]) = bigendians(*((uint16_t*)&servopacket->chn[1]));
   //servopacket.chn[2] = ((tmpr = buffer[37]) << 8)|buffer[38]; tmpr = 0;
   *((uint16_t*)&buffer[ 6]) = bigendians(*((uint16_t*)&servopacket->chn[2]));
   //servopacket.chn[3] = ((tmpr = buffer[39]) << 8)|buffer[40]; tmpr = 0;
   *((uint16_t*)&buffer[ 8]) = bigendians(*((uint16_t*)&servopacket->chn[3]));
   //servopacket.chn[4] = ((tmpr = buffer[41]) << 8)|buffer[42]; tmpr = 0;
   *((uint16_t*)&buffer[10]) = bigendians(*((uint16_t*)&servopacket->chn[4]));
   //servopacket.chn[5] = ((tmpr = buffer[43]) << 8)|buffer[44]; tmpr = 0;
   *((uint16_t*)&buffer[12]) = bigendians(*((uint16_t*)&servopacket->chn[5]));
   //servopacket.chn[6] = ((tmpr = buffer[45]) << 8)|buffer[46]; tmpr = 0;
   *((uint16_t*)&buffer[14]) = bigendians(*((uint16_t*)&servopacket->chn[6]));
   //servopacket.chn[7] = ((tmpr = buffer[47]) << 8)|buffer[48]; tmpr = 0; 
   *((uint16_t*)&buffer[16]) = bigendians(*((uint16_t*)&servopacket->chn[7]));
}

/***************************************************************************************
 *encode the imu data packet
 ***************************************************************************************/
void MNAV::encode_packet(struct imu *data, struct gps *gpsdata, struct servo *servopacket, uint8_t* buffer)
{
   //signed short tmp=0,i=0;
   //unsigned short tmpr=0;
   signed short tmp16;
   short i;
   uint16_t  sum=0;
   
   buffer[ 0] = 0x55;
   buffer[ 1] = 0x55;
   buffer[ 2] = OUTPUT_PACKET_TYPE;

   /* acceleration in m/s^2 */
   //data->ax = (double)(((tmp = (signed char)buffer[ 3])<<8)|buffer[ 4])*5.98754883e-04; tmp=0;
   tmp16 = (signed short)(data->ax*0.1670132517315938e04); *((uint16_t*)&buffer[ 3]) = bigendians(*((uint16_t*)&tmp16));
   //data->ay = (double)(((tmp = (signed char)buffer[ 5])<<8)|buffer[ 6])*5.98754883e-04; tmp=0;
   tmp16 = (signed short)(data->ay*0.1670132517315938e04); *((uint16_t*)&buffer[ 5]) = bigendians(*((uint16_t*)&tmp16));
   //data->az = (double)(((tmp = (signed char)buffer[ 7])<<8)|buffer[ 8])*5.98754883e-04; tmp=0;
   tmp16 = (signed short)(data->az*0.1670132517315938e04); *((uint16_t*)&buffer[ 7]) = bigendians(*((uint16_t*)&tmp16));
  

   /* angular rate in rad/s */
   //data->p  = (double)(((tmp = (signed char)buffer[ 9])<<8)|buffer[10])*1.065264436e-04; tmp=0;
   tmp16 = (signed short)(data->p*0.9387340515702713e04); *((uint16_t*)&buffer[ 9]) = bigendians(*((uint16_t*)&tmp16));
   //data->q  = (double)(((tmp = (signed char)buffer[11])<<8)|buffer[12])*1.065264436e-04; tmp=0;
   tmp16 = (signed short)(data->q*0.9387340515702713e04); *((uint16_t*)&buffer[11]) = bigendians(*((uint16_t*)&tmp16));
   //data->r  = (double)(((tmp = (signed char)buffer[13])<<8)|buffer[14])*1.065264436e-04; tmp=0;
   tmp16 = (signed short)(data->r*0.9387340515702713e04); *((uint16_t*)&buffer[13]) = bigendians(*((uint16_t*)&tmp16));
   
   /* magnetic field in Gauss */
   //data->hx = (double)(((tmp = (signed char)buffer[15])<<8)|buffer[16])*6.103515625e-05; tmp=0;
   tmp16 = (signed short)(data->hx*0.16384e05); *((uint16_t*)&buffer[15]) = bigendians(*((uint16_t*)&tmp16));
   //data->hy = (double)(((tmp = (signed char)buffer[17])<<8)|buffer[18])*6.103515625e-05; tmp=0;
   tmp16 = (signed short)(data->hy*0.16384e05); *((uint16_t*)&buffer[17]) = bigendians(*((uint16_t*)&tmp16));
   //data->hz = (double)(((tmp = (signed char)buffer[19])<<8)|buffer[20])*6.103515625e-05; tmp=0;
   tmp16 = (signed short)(data->hz*0.16384e05); *((uint16_t*)&buffer[19]) = bigendians(*((uint16_t*)&tmp16));

   /* temperature in Celcius */
   /*
   data->Tx = (double)(((tmp = (signed char)buffer[21])<<8)|buffer[22])*6.103515625e-03; tmp=0;
   data->Ty = (double)(((tmp = (signed char)buffer[23])<<8)|buffer[24])*6.103515625e-03; tmp=0;
   data->Tz = (double)(((tmp = (signed char)buffer[25])<<8)|buffer[26])*6.103515625e-03; tmp=0;
   */
   buffer[21] = 0;
   buffer[22] = 0;
   buffer[23] = 0;
   buffer[24] = 0;
   buffer[25] = 0;
   buffer[26] = 0;   
   
   /* pressure in m and m/s */
   //data->Ps = (double)(((tmp = (signed char)buffer[27])<<8)|buffer[28])*3.0517578125e-01; tmp=0;
   tmp16 = (signed short)(data->Ps*0.32768e01); *((uint16_t*)&buffer[27]) = bigendians(*((uint16_t*)&tmp16));
   //data->Pt = (double)(((tmp = (signed char)buffer[29])<<8)|buffer[30])*2.4414062500e-03; tmp=0;
   tmp16 = (signed short)(data->Pt*0.4096e03); *((uint16_t*)&buffer[29]) = bigendians(*((uint16_t*)&tmp16));


   /* servo packet */
   switch (buffer[2]) {
      case 'S' :   
                   #if 0
                   //servopacket.status = buffer[32];
		   buffer[32] = servopacket->status;
   		   //servopacket.chn[0] = ((tmpr = buffer[33]) << 8)|buffer[34]; tmpr = 0;
   		   *((uint16_t*)&buffer[33]) = bigendians(*((uint16_t*)&servopacket->chn[0]));
	           //servopacket.chn[1] = ((tmpr = buffer[35]) << 8)|buffer[36]; tmpr = 0;
	           *((uint16_t*)&buffer[35]) = bigendians(*((uint16_t*)&servopacket->chn[1]));
		   //servopacket.chn[2] = ((tmpr = buffer[37]) << 8)|buffer[38]; tmpr = 0;
		   *((uint16_t*)&buffer[37]) = bigendians(*((uint16_t*)&servopacket->chn[2]));
		   //servopacket.chn[3] = ((tmpr = buffer[39]) << 8)|buffer[40]; tmpr = 0;
		   *((uint16_t*)&buffer[39]) = bigendians(*((uint16_t*)&servopacket->chn[3]));
		   //servopacket.chn[4] = ((tmpr = buffer[41]) << 8)|buffer[42]; tmpr = 0;
		   *((uint16_t*)&buffer[41]) = bigendians(*((uint16_t*)&servopacket->chn[4]));
		   //servopacket.chn[5] = ((tmpr = buffer[43]) << 8)|buffer[44]; tmpr = 0;
		   *((uint16_t*)&buffer[43]) = bigendians(*((uint16_t*)&servopacket->chn[5]));
		   //servopacket.chn[6] = ((tmpr = buffer[45]) << 8)|buffer[46]; tmpr = 0;
		   *((uint16_t*)&buffer[45]) = bigendians(*((uint16_t*)&servopacket->chn[6]));
		   //servopacket.chn[7] = ((tmpr = buffer[47]) << 8)|buffer[48]; tmpr = 0; 
		   *((uint16_t*)&buffer[47]) = bigendians(*((uint16_t*)&servopacket->chn[7]));
                   #endif
                   encode_servopacket(servopacket, &buffer[31]);
                   
                   for(i=2;i<49;i++) sum += buffer[i];
		   buffer[49] = (uint8_t)(sum >> 8);
		   buffer[50] = (uint8_t)sum;
		   break;
      case 'N' :   
                   #if 0//servopacket.status = buffer[67];
		   buffer[67] = servopacket->status;
   		   //servopacket.chn[0] = ((tmpr = buffer[68]) << 8)|buffer[69]; tmpr = 0;
   		   *((uint16_t*)&buffer[68]) = bigendians(*((uint16_t*)&servopacket->chn[0]));
	           //servopacket.chn[1] = ((tmpr = buffer[70]) << 8)|buffer[71]; tmpr = 0;
	           *((uint16_t*)&buffer[70]) = bigendians(*((uint16_t*)&servopacket->chn[1]));
		   //servopacket.chn[2] = ((tmpr = buffer[72]) << 8)|buffer[73]; tmpr = 0;
		   *((uint16_t*)&buffer[72]) = bigendians(*((uint16_t*)&servopacket->chn[2]));
		   //servopacket.chn[3] = ((tmpr = buffer[74]) << 8)|buffer[75]; tmpr = 0;
		   *((uint16_t*)&buffer[74]) = bigendians(*((uint16_t*)&servopacket->chn[3]));
		   //servopacket.chn[4] = ((tmpr = buffer[76]) << 8)|buffer[77]; tmpr = 0;
		   *((uint16_t*)&buffer[76]) = bigendians(*((uint16_t*)&servopacket->chn[4]));
		   //servopacket.chn[5] = ((tmpr = buffer[78]) << 8)|buffer[79]; tmpr = 0;
		   *((uint16_t*)&buffer[78]) = bigendians(*((uint16_t*)&servopacket->chn[5]));
		   //servopacket.chn[6] = ((tmpr = buffer[80]) << 8)|buffer[81]; tmpr = 0;
		   *((uint16_t*)&buffer[80]) = bigendians(*((uint16_t*)&servopacket->chn[6]));
		   //servopacket.chn[7] = ((tmpr = buffer[82]) << 8)|buffer[83]; tmpr = 0;
		   *((uint16_t*)&buffer[82]) = bigendians(*((uint16_t*)&servopacket->chn[7]));
                   #endif
                   encode_gpspacket(gpsdata, &buffer[31]);
                   encode_servopacket(servopacket, &buffer[66]);
                   
                   for(i=2;i<84;i++) sum += buffer[i];
		   buffer[84] = (uint8_t)(sum >> 8);
		   buffer[85] = (uint8_t)sum;
                   break;
      case 'I' :
                   encode_gpspacket(gpsdata, &buffer[31]);
                   encode_ahrspacket(data, &buffer[66]);
                   encode_servopacket(servopacket, &buffer[73]);
                   
                   for(i=2;i<91;i++) sum += buffer[i];
		   buffer[91] = (uint8_t)(sum >> 8);
		   buffer[92] = (uint8_t)sum;
                   break;
      default  :
                   fprintf(stderr, "[imu]:fail to encode servo packet..!\n");
   }

  
   //data->time = get_Time();
   //data->err_type = no_error;
     
}


int MNAV::get_servo_cmd(uint16_t cnt_cmd[3], uint8_t *reverse)
{
  int		count=0,nbytes=0;
  uint8_t  	input_buffer[SERVO_PACKET_SIZE]={0,};
  
  read_into_buffer();

  while (1) {
  /*********************************************************************
   *Find start of packet: the heade r (2 bytes) starts with 0x5555
   *********************************************************************/
  while(circbufLength >= 4 && (circbuf[circbufStart] != (uint8_t)0x55 || circbuf[(uint8_t)(circbufStart + 1)] != (uint8_t)0x55))
  {
      circbufStart++;
      circbufLength--;
  }
  if(circbufLength < 4)
      return count;

  /*********************************************************************
   *Read packet contents
   *********************************************************************/
  switch (circbuf[(uint8_t)(circbufStart + 2)])
  {
	case 'S':
		  if(circbuf[(uint8_t)(circbufStart + 3)] == 'F')
		  {
		    //fprintf(stderr, "Got 'S' 'F'\n");
		    circbufStart += circbufLength >= (uint8_t)11 ? (uint8_t)11 : circbufLength;
		    circbufLength -= circbufLength >= (uint8_t)11 ? (uint8_t)11 : circbufLength;
		    break;
		  }
		  
		  if(circbuf[(uint8_t)(circbufStart + 3)] == 'P')
		  {
		    if(circbufLength < 7)
		      return count;
		    //fprintf(stderr, "Got 'S' 'P'\n");
		    for(nbytes = 0; nbytes < 7; nbytes++)
		    {
		      input_buffer[nbytes] = circbuf[circbufStart];
		      circbufStart++;
		      circbufLength--;
		    }
		    if(checksum(input_buffer, 7)) 
		      *reverse = input_buffer[4]; 
		    else 
		      fprintf(stderr, "'S' 'P' does not pass checksum\n");
		    break;
		  }
		  
		  if(circbuf[(uint8_t)(circbufStart + 3)] != 'S')
		  {
		    //fprintf(stderr, "Got unknown 'S' '%c'\n", circbuf[(uint8_t)(circbufStart + 3)]);
		    circbufStart += (uint8_t)4;
		    circbufLength -= (uint8_t)4;
		    break;
		  }
		  
		  if(circbufLength < SERVO_PACKET_SIZE)
		    return count;
		  for(nbytes = 0; nbytes < SERVO_PACKET_SIZE; nbytes++)
		  {
		    input_buffer[nbytes] = circbuf[circbufStart];
		    circbufStart++;
		    circbufLength--;
		  }

  		  /*************************
                   *check checksum
                   *************************/
                  if(checksum(input_buffer,SERVO_PACKET_SIZE))
		  {
	             //pthread_mutex_lock(&mutex_imu);
		       //decode_imupacket(&imupacket, input_buffer);
		       decode_servo_cmd(input_buffer, cnt_cmd);
		       count++;
		       //if(screen_on) fwrite(&imupacket, sizeof(struct imu),1,fimu);
		       //pthread_cond_signal(&trigger_ahrs);
		     //pthread_mutex_unlock(&mutex_imu);
		  }
                  else
                    fprintf(stderr, "Servo command does not pass checksum\n");
		  break;
    
	default:
		  circbufStart += 3;
		  circbufLength -= 3;
		  break;
  }
  
  } 
  
  return count;
}

void MNAV::decode_servo_cmd(uint8_t data[24], uint16_t cnt_cmd[3])
{
   //cnt_cmd[1] = ch1:elevator, cnt_cmd[0] = ch0:aileron, cnt_cmd[2] = ch2:throttle
   //byte  data[24]={0,};
   //short i = 0, nbytes = 0;
   //word  sum=0;

   //data[0] = 0x55; 
   //data[1] = 0x55;
   //data[2] = 0x53;
   //data[3] = 0x53;

   //elevator
   //data[6] = (byte)(cnt_cmd[1] >> 8);
   //data[7] = (byte)cnt_cmd[1];
   cnt_cmd[1] = ((uint16_t)data[6] << 8) | (uint16_t)data[7];
   //throttle
   //data[8] = (byte)(cnt_cmd[2] >> 8);
   //data[9] = (byte)cnt_cmd[2];
   cnt_cmd[2] = ((uint16_t)data[8] << 8) | (uint16_t)data[9];
   //aileron
   //data[4] = (byte)(cnt_cmd[0] >> 8); 
   //data[5] = (byte)cnt_cmd[0];
   cnt_cmd[0] = ((uint16_t)data[4] << 8) | (uint16_t)data[5];
   
   //checksum:need to be verified
   //sum = 0xa6;
   //for(i=4;i<22;i++) sum += data[i];
  
   //data[22] = (byte)(sum >> 8);
   //data[23] = (byte)sum;

   //sendout the command packet
   //while (nbytes != 24) nbytes = write(sPort0,(char*)data, 24);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// display message
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void MNAV::display_message(struct imu *data, struct gps *gdata)
{
   //static int count=0;
   	
   //if (++count == disptime)
   //{
	printf("[m/s^2]:ax  = %6.3f ay  = %6.3f az  = %6.3f \n",data->ax,data->ay,data->az);
	printf("[deg/s]:p   = %6.3f q   = %6.3f r   = %6.3f \n",data->p*57.3, data->q*57.3, data->r*57.3);
        printf("[deg  ]:phi = %6.2f the = %6.2f psi = %6.2f \n",data->phi*57.3,data->the*57.3,data->psi*57.3);
	printf("[Gauss]:hx  = %6.3f hy  = %6.3f hz  = %6.3f \n",data->hx,data->hy,data->hz);
        printf("[press]:Ps  = %f[m] Pt  = %f\n", data->Ps, data->Pt);
        //printf("[deg/s]:bp  = %6.3f,bq  = %6.3f,br  = %6.3f \n\n",xvar[4][0]*57.3,xvar[5][0]*57.3,zvar[1][0]*57.3);
        //if (ndata->err_type == TRUE) {
          printf("[GPS  ]:ITOW= %5d[ms], lon = %f[deg], lat = %f[deg], alt = %f[m]\n",gdata->ITOW,gdata->lon,gdata->lat,gdata->alt);	
          printf("[GPS  ]:ve = %6.3f[m/s],vn = %6.3f[m/s],vd = %6.3f[m/s]\n",gdata->ve,gdata->vn,gdata->vd);
          //printf("[nav  ]:                 lon = %f[deg], lat = %f[deg], alt = %f[m]\n",            ndata->lon,ndata->lat,ndata->alt);	
	//}

	//count = 0;
   //}	
   printf("\n");
}
