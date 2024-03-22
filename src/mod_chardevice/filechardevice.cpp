//----------------------------------------------------------------------------------//
//                                                                                  //
//  Created on November 09, 2006                                                    //
//                                                                                  //
//  "Copyright (c) 2006 The Regents of the University of California.                //
//  All rights reserved.                                                            //
//                                                                                  //
//  Permission to use, copy, modify, and distribute this software and its           //
//  documentation for any purpose, without fee, and without written agreement is    //
//  hereby granted, provided that the above copyright notice, the following         //
//  two paragraphs and the author appear in all copies of this software.            //
//                                                                                  //
//  IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO ANY PARTY FOR       //
//  DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT     //
//  OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF    //
//  CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                  //
//                                                                                  //
//  THE UNIVERSITY OF CALIFORNIA SPECIFICALLY DISCLAIMS ANY WARRANTIES,             //
//  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY        //
//  AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS       //
//  ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION TO      //
//  PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS."         //
//                                                                                  //
//----------------------------------------------------------------------------------//


//----------------------------------------------------------------------------------
//
// filechardevice.cpp
//
//    Cross-platform interface to files (one for reading, and another for writing).
//
// authors
//
//    Todd R. Templeton <ttemplet@eecs.berkeley.edu>
//
//----------------------------------------------------------------------------------


#include "filechardevice.h"
#include "chardevicecommon.h"


// open device, called first time by constructor (uses wait, returns -1 on error)
int FileCharDevice::open( void )
{
    int i;
    int len = strlen( options );
    char *filenameRead;
    char *filenameWrite;

    // find input file name
    for( i = 0; i < len && options[ i ] != ','; i++ );
    options[ i ] = '\0';
    filenameRead = i > 0 ? options : 0;
    // find output file name
    filenameWrite = ( i < len - 1 ) ? &options[ i + 1 ] : 0;

    // open files
    if( filenameRead )
    {
        fprintf( stderr, "Opening file %s for reading.\n", filenameRead );
        fdRead = fopen( filenameRead, "rb" );
    }
    else
        fdRead = 0;
    if( ( fdRead != 0 || !filenameRead ) && filenameWrite )
    {
        fprintf( stderr, "Opening file %s for writing.\n", filenameWrite );
        fdWrite = fopen( filenameWrite, "wb" );
    }
    else
        fdWrite = 0;
    
    // repair options
    options[ i ] = ',';
    
    return ( ( fdRead == 0 && filenameRead ) || ( fdWrite == 0 && filenameWrite ) ) ? -1 : 0;
}

// read from device (buf = buffer into which to read, count = max number of bytes to read,
//   uses maxInterval and wait, returns number of bytes read or -1 on error)
ssize_t FileCharDevice::read( void *buf, size_t count )
{
    return ( fdRead != 0 ) ? fread( buf, 1, count, fdRead ) : 0;
}

// write to device (buf = buffer from which to write, count = number number of bytes to write,
//   uses wait, returns number of bytes written or -1 on error)
ssize_t FileCharDevice::write( const void *buf, size_t count )
{
    ssize_t retval = 0;
    if( fdWrite != 0 )
    {
        retval = fwrite( buf, 1, count, fdWrite );
        fflush( fdWrite );
    }
    return retval;
}

// close device, called last time by destructor (returns 0 on success or -1 on error)
int FileCharDevice::close( void )
{
    int retval = 0;
    if( fdRead != 0 )
    {
        if( fclose( fdRead ) == EOF )
            retval = -1;
        fdRead = 0;
    }
    if( fdWrite != 0 )
    {
        if( fclose( fdWrite ) == EOF )
            retval = -1;
        fdWrite = 0;
    }
    return retval;
}
