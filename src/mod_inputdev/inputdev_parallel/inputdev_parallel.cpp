/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2001 - Jan Edward Kansky (original author)
 *   Copyright (C) 2004, 2005, 2008 - Jens Wilhelm Wulf
 *   Copyright (C) 2005, 2007, 2008 - Jan Reucker
 *   Copyright (C) 2004 - Lionel Cailler
 *   Copyright (C) 2006 - Todd Templeton
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
#include <crrc_config.h>
#include "inputdev_parallel.h"

/** Include for ioperm and parallel port access, depends on version of libc */
#if defined(__APPLE__) || defined(MACOSX)
#else
#  if defined(WIN32)
#  elif defined(__powerpc__)
#    define ioperm(a,b,c) -1
#    define inb(a) 0
#    define outb(a,b)
#  else   // Linux
#    ifdef HAVE_SYS_IO_H
#     include <sys/io.h>
#    else
#     include <unistd.h>
#     include <asm/io.h>
#    endif
#  endif
#endif  // __APPLE__

#ifdef linux
 #include <time.h>
 #include <sys/time.h>
#endif

#if defined(__APPLE__) || defined(MACOSX)
 #include <time.h>
 #include <sys/time.h>
#endif  // __APPLE__


#include <stdio.h>
#include <plib/ul.h>    // for dynamic library loading

#ifdef WIN32
#include <windows.h>
// replacement for dlportio.h
//~ extern "C" {
  //~ unsigned char DlPortReadPortUchar(unsigned int);
  //~ void          DlPortWritePortUchar(unsigned long, unsigned char);
//~ }
#endif

typedef unsigned char (*ReadUCharFunc)(unsigned int);
typedef void (*WriteUCharFunc)(unsigned long, unsigned char);

// handler for dlportio.dll
ulDynamicLibrary *dlportio      = NULL;
ReadUCharFunc     pfReadUChar   = NULL;
WriteUCharFunc    pfWriteUChar  = NULL;


/** \brief Basic initialization for parallel interface
 *
 *  This routine must be called once at startup time to load
 *  the shared library needed to access the parallel port.
 *  
 *  On Win32 we'll try to load dlportio.dll and get function
 *  pointers to the in and out functions. If we don't succeed,
 *  the pointer to the handler class will remain at NULL.
 *  The parallel interface won't work in this case.
 *
 *  On all other platforms we don't do anything, which means that
 *  the handler class pointer is always NULL.
 */
void ParallelInterfaceBaseInit()
{
  #ifdef WIN32
  dlportio = new ulDynamicLibrary("dlportio");
  pfReadUChar  = (ReadUCharFunc)dlportio->getFuncAddress("DlPortReadPortUchar");
  pfWriteUChar = (WriteUCharFunc)dlportio->getFuncAddress("DlPortWritePortUchar");
  
  if ((pfReadUChar == NULL) || (pfWriteUChar == NULL))
  {
    fprintf(stderr, "ParallelInterfaceBaseInit: Unable to load symbols from dlportio.dll!\n");
    fprintf(stderr, "Parallel interface will be disabled.\n");
    fprintf(stderr, "Please consult the CRRCsim documentation and install dlportio.dll!\n");
    delete dlportio;
    dlportio      = NULL;
    pfReadUChar   = NULL;
    pfWriteUChar  = NULL;
  }
  #endif
}


#ifndef linux
/***************************************************************************/
unsigned char inb(unsigned int PORT)
{
  unsigned char buf = 0;
  if (pfReadUChar != NULL)
  {
    buf = pfReadUChar(PORT);
  }
  return buf;
}

/***************************************************************************/
void outb(unsigned char output_value, unsigned int PORT)
{ 
  if (pfWriteUChar != NULL)
  {
    pfWriteUChar(PORT,output_value);
  }
}
#endif



T_TX_InterfaceParallel::T_TX_InterfaceParallel()
  : cname("parallel")
{
  printf("T_TX_InterfaceParallel::T_TX_InterfaceParallel()\n");
  lpt_calibration_factor=1158726;
  ParallelInterfaceBaseInit();
}


T_TX_InterfaceParallel::~T_TX_InterfaceParallel()
{
}


int T_TX_InterfaceParallel::init(SimpleXMLTransfer* config)
{
  printf("int T_TX_InterfaceParallel::init(SimpleXMLTransfer* config)\n");

  T_TX_InterfacePPM::init(config);

  mixer = new T_TX_Mixer(this, config, "inputMethod.parallel");

  nParPortNum = config->getInt("inputMethod.parallel.lpt", 1);
  
  return(init_parallel_interface(nParPortNum));
}


void T_TX_InterfaceParallel::getInputData(TSimInputs* inputs)
{
  get_data_from_parallel_interface(rc_channel_values);
  T_TX_InterfacePPM::getInputData(inputs);
}


void T_TX_InterfaceParallel::putBackIntoCfg(SimpleXMLTransfer* config)
{
  T_TX_InterfacePPM::putBackIntoCfg(config);

  config->setAttributeOverwrite("inputMethod.parallel.lpt", nParPortNum);
}


/***************************************************************************/
void T_TX_InterfaceParallel::calibrate_parallel_interface()
  //  Determine how many ticks per second elapse in the microcontroller so
  // that the output can be calibrated to absolute time.
{
  unsigned char data;

#ifdef WIN32
  __int64 frame1s, start, end;
#else // WIN32
  struct timeval s_time,e_time;
  struct timezone tz;
#endif
  unsigned int int_data,new_data = 0;
  int cycles,channel_num,current_nibble = 0;
  float total_clocks,lpt_elapsed_time;
  int num_calibration_cycles;
  unsigned int channels[11];
  float total_clocks_array[11];
  int loop;

  for (loop=0;loop<11;loop++)
  {
    total_clocks_array[loop]=0;
    channels[loop]=0;
  }
  num_calibration_cycles=150;
  cycles=0;
  do
  {
    ask_for_data();
    wait_until_data_ready(&data);
    acknowledge_got_data();
    wait_until_interface_is_idle(&data);
  }
  while((data & 0x08) == 0);
  channel_num=1;
#ifdef WIN32
  QueryPerformanceFrequency((LARGE_INTEGER*)&frame1s);
  QueryPerformanceCounter((LARGE_INTEGER*)&start);
#else // WIN32
  gettimeofday(&s_time,&tz);     // Get time that new data arrived.
#endif
  while (cycles<num_calibration_cycles)       // Get 150 new frames of data
  {
    ask_for_data();
    wait_until_data_ready(&data);
    acknowledge_got_data();
    if (data & 0x80)
    {
      data&=0xBF;
    }
    else
    {
      data|=0x40;
    }
    data=(data&0x78)>>3;

    if (channel_num != 0)
    {
      int_data=(unsigned int)data;
      new_data|=(int_data<<(current_nibble*4));
      current_nibble++;
      if (current_nibble==3)
      {
        channels[channel_num-1]=new_data;
        current_nibble=0;
        new_data=0;
        if (channel_num < 11)
        {
          channel_num++;
        }
        else
        {
          printf("RC Transmitter Interface Read Error.\n");
        }
        if (channel_num==10)
        {
          total_clocks_array[0]+=channels[0];
          total_clocks_array[1]+=channels[1];
          total_clocks_array[2]+=channels[2];
          total_clocks_array[3]+=channels[3];
          total_clocks_array[4]+=channels[4];
          total_clocks_array[5]+=channels[5];
          total_clocks_array[6]+=channels[6];
          total_clocks_array[7]+=channels[7];
          total_clocks_array[8]+=channels[8];
          total_clocks_array[9]+=channels[9];
        }
      }
    }
    wait_until_interface_is_idle(&data);
    if (data & 0x08)
    {
      cycles++;
      if (cycles==num_calibration_cycles)
      {
#ifdef WIN32
        QueryPerformanceCounter((LARGE_INTEGER*)&end);
#else // WIN32
        gettimeofday(&e_time,&tz);
#endif
      }
      current_nibble=0;
      new_data=0;
      channel_num=1;
    }
  }
#ifdef WIN32
  lpt_elapsed_time=((float)(end - start))/frame1s;
#else // WIN32
  lpt_elapsed_time=((((float)e_time.tv_sec)*1E6+((float)e_time.tv_usec))-
                    (((float)s_time.tv_sec*1E6)+((float)s_time.tv_usec)))/1E6;
#endif

  for (loop=0;loop<9;loop++)
  {
    printf("%f ",total_clocks_array[loop]);
  }
  printf("\n");

  total_clocks=0;
  for (loop=0;loop<8;loop++)
  {
    total_clocks+=total_clocks_array[loop];
  }
  total_clocks+=total_clocks_array[9]*8;
  printf("%f\n",total_clocks);
  lpt_calibration_factor=(total_clocks/lpt_elapsed_time);
  printf("Parallel Port Micro-controller Clock Frequency: %f MHz\n",
         lpt_calibration_factor/1E6);
}

/***************************************************************************/
int T_TX_InterfaceParallel::init_parallel_interface(int lpt)
{
  unsigned char data;

  #ifndef linux
  if (dlportio == NULL)
  {
    #ifdef WIN32
    errMsg = "Sorry, unable to load the required library DLPORTIO.DLL\n"
             "Parallel interface not available.";
    #else
    errMsg = "Sorry, parallel interface is not available on this operating system.";
    #endif
    return -1;
  }
  #endif

  if (lpt == 1)
  {
    LPT_DATA=0x378;
  }
  else if (lpt==2)
  {
    LPT_DATA=0x278;
  }
  else if (lpt==3)
  {
    LPT_DATA=0x3F8;
  }
  LPT_STATUS=LPT_DATA+1;
  LPT_CONTROL=LPT_DATA+2;
  
#ifdef linux
  if (ioperm(LPT_DATA,3,1))
  {
    errMsg = "Sorry, you were not able to gain access to the ports\n"
             "You must be root to run this program";
    printf("error: %s\n", errMsg.c_str());
    return(-1);
  }
#endif

  do
  {
    ask_for_data();
    wait_until_data_ready(&data);
    if (errMsg.length())
      return(-1);
    
    acknowledge_got_data();
    wait_until_interface_is_idle(&data);
    if (errMsg.length())
      return(-1);
  }
  while((data & 0x08) == 0);
  printf("Received parallel interface sync pulse.\n");
  printf("Calibrating Parallel Port micro-controller clock frequency.\n");
  calibrate_parallel_interface();
  printf("Parallel port interface calibrated.\n");
  
  return(0);
}

/***************************************************************************/
void T_TX_InterfaceParallel::get_data_from_parallel_interface(float *rc_channel_values)
  //  Check to see if there is new data waiting on the parallel interface.
  // If there is, then get a nibblet
{
  unsigned char data;
  unsigned int int_data,new_data = 0;
  int synched,channel_num,num_read,current_nibble;
  //int loop;  
  //unsigned int channels[11];

  num_read=0;
  synched=0;
  current_nibble=0;
  channel_num=1;
  //for (loop=0;loop<11;loop++)
  //{
  //  channels[loop]=0;
  //}
  data=inb(LPT_STATUS);
  if ((data & 0x40)!=0)
  {
    data=inb(LPT_STATUS);
    if (data & 0x08)
    {
      current_nibble=0;
      new_data=0;
      channel_num=1;
    }
    do
    {
      ask_for_data();
      wait_until_data_ready(&data);
      acknowledge_got_data();
      if (data & 0x80)
      {
        data&=0xBF;
      }
      else
      {
        data|=0x40;
      }
      data=(data&0x78)>>3;
      if (channel_num != 0)
      {
        int_data=(unsigned int)data;
        new_data|=(int_data<<(current_nibble*4));
        current_nibble++;
        num_read++;
        if (current_nibble==3)
        {
          //channels[channel_num-1]=new_data;
          rc_channel_values[channel_num-1]=((float)new_data*1000/
                                            lpt_calibration_factor)-1.52;
          current_nibble=0;
          new_data=0;
          if (channel_num < 11)
          {
            channel_num++;
          }
          else
          {
            printf("RC Transmitter Interface Read Error.\n");
          }
        }
      }
      if (num_read < 30)
      {
        wait_until_interface_is_idle(&data);
        if (data & 0x08)
        {
          current_nibble=0;
          new_data=0;
          channel_num=1;
          synched=1;
        }
      }
    }
    while ((synched==0) && (num_read < 30));
  }
}

/***************************************************************************/
void T_TX_InterfaceParallel::ask_for_data()
// Set HostBusy low to indicate data desired, inverted logic.
{
 unsigned char data;

 data=inb(LPT_CONTROL);
 outb(0x02|data,LPT_CONTROL);
}

/***************************************************************************/
void T_TX_InterfaceParallel::acknowledge_got_data()
// Set HostBusy high indicating that the data has been read, inverted logic.
{
 unsigned char data;

 data=inb(LPT_CONTROL);
 outb(0xFD&data,LPT_CONTROL);
}

/**
 * todo: timeout recognized by counting loops only. 
 * 
 * Wait until the S6 bit is brought low, indicating data is ready to be read.
 */
void T_TX_InterfaceParallel::wait_until_data_ready(unsigned char *data)
{
  int num_loops;

  num_loops=0;
  do
  {
    (*data)=inb(LPT_STATUS);
    num_loops++;
  }
  while (((*data) & 0x40)!=0 && (num_loops < 25000));
  if (num_loops >=25000)
  {
    printf("Communications time-out with RC transmitter interface.\n");
    errMsg = "Communications time-out with RC transmitter interface.";
  }
}


/**
 * todo: timeout recognized by counting loops only.
 * 
 * Wait until the S6 bit is brought high, indicating interface is idle and
 * there is more data to send.
 */
void T_TX_InterfaceParallel::wait_until_interface_is_idle(unsigned char *data)
{
  int num_loops;
  
  num_loops=0;
  do
  {
    (*data)=inb(LPT_STATUS);
    num_loops++;
  }
  while (((*data) & 0x40)==0 && (num_loops < 25000));
  if (num_loops >=25000)
  {
    printf("Communications time-out with RC transmitter interface.\n");
    errMsg = "Communications time-out with RC transmitter interface.";
  }
  (*data)=inb(LPT_STATUS);
}
