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
// chardevicewrapper.h
//
//    A "wrapper" chardevice that allows any one of the other chardevices to be used
//    transparently.
//
// authors
//
//    Todd R. Templeton <ttemplet@eecs.berkeley.edu>
//
//----------------------------------------------------------------------------------


#ifndef __CHARDEVICEWRAPPER_H__
#define __CHARDEVICEWRAPPER_H__

#include "chardevicebase.h"


// character device for wrapping/logging communication using any of the above devices
class CharDeviceWrapper : public CharDevice
{
protected:

    // helper class for writing to a log
    class LogWriter
    {
    private:
    
        // char device in use
        CharDevice *charDevice;
    
            
    public:
    
        // write to log
        void write_log( uint64_t time, const void *buf, size_t count );
        
        // constructor
        LogWriter( CharDevice *_charDevice ) : charDevice( _charDevice ) {}
        
        // destructor
        ~LogWriter( ) {}
    };
    
    
    // helper class for reading from a log
    class LogReader
    {
    private:
    
        // char device in use
        CharDevice *charDevice;
        // log item header
        char header[ 8 ];
        // accounting
        int headerBytesRead;
        uint64_t logItemTime;
        uint16_t logItemBytes;
        uint16_t logItemBytesRead;
        
            
    public:
    
        // read from log
        ssize_t read_log( uint64_t time, const void *buf, size_t count );
        
        // constructor
        LogReader( CharDevice *_charDevice ) : charDevice( _charDevice ), headerBytesRead( 0 ) {}
        
        // destructor
        ~LogReader( ) {}
    };
    
    
    // char devices in use
    CharDevice *charDevice;
    CharDevice *readLogCharDevice;
    CharDevice *writeLogCharDevice;
    // private copy of options strings
    char *options;
    char *readLogOptions;
    char *writeLogOptions;
    // whether to read from the read log (instead of normal operation)
    bool readFromLog;
    // log reader/writers
    LogReader *logReader;
    LogWriter *readLogWriter;
    LogWriter *writeLogWriter;
    bool loggingOn;
    
    
public:

    // open device, called first time by constructor (uses wait, returns -1 on error)
    int open( void );
    // read from device (buf = buffer into which to read, count = max number of bytes to read,
    //   uses maxInterval and wait, returns number of bytes read or -1 on error)
    ssize_t read( void *buf, size_t count );
    // write to device (buf = buffer from which to write, count = number number of bytes to write,
    //   uses wait, returns number of bytes written or -1 on error)
    ssize_t write( const void *buf, size_t count );
    // close device, called last time by destructor (returns 0 on success or -1 on error)
    int close( void );
    
    
    // set whether to wait for connection/reconnection
    void set_wait_for_connection( bool _wait )
    {
        wait = _wait;
        if( charDevice )
            charDevice->set_wait_for_connection( _wait );
    }
    // set time in seconds since last successful read after which to determine that
    //   link has gone down
    void set_max_read_interval( double _maxInterval )
    {
        maxInterval = _maxInterval;
        if( charDevice )
            charDevice->set_max_read_interval( _maxInterval );
    }
    // set whether to wait for at least one byte before returning from a read
    void set_wait_for_data( bool _waitForData )
    {
        waitForData = _waitForData;
        if( charDevice )
            charDevice->set_wait_for_data( _waitForData );
        if( readFromLog && readLogCharDevice )
            readLogCharDevice->set_wait_for_data( _waitForData );
    }
    
    
    // enable logging
    void enable_logging( void )
    {
        if( readLogWriter || writeLogWriter )
            loggingOn = true;
    }
    // disable logging
    void disable_logging( void )
    {
        loggingOn = false;
    }
    // return whether logging is enabled
    bool is_logging_enabled( void )
    {
        return loggingOn;
    }


    // run script from file (returns -1 on error)
    int run_script( const char *scriptfile );
    
    
    // set function call for getting the current time (in microseconds)
    void set_time_function( uint64_t (*_get_time)( void ) )
    {
        get_time = _get_time;
        if( charDevice )
            charDevice->set_time_function( _get_time );
        if( readLogCharDevice )
            readLogCharDevice->set_time_function( _get_time );
        if( writeLogCharDevice )
            writeLogCharDevice->set_time_function( _get_time );
    }
    

    // constructor (_options = options string, _wait = whether to wait for connection/reconnection, _readFromLog = whether to read from the read log (instead of normal operation))
    CharDeviceWrapper( const char *_options, bool _wait = true ) : charDevice( 0 ), readFromLog( false ), loggingOn( false )
    {
        int i, i2;
        int len = strlen( _options );

        this->set_wait_for_connection( _wait );

        // find options
        for( i = 0, i2 = 0; i < len && _options[ i ] != ';'; i++ );
        if( i - i2 > 1 )
        {
            options = new char[ i - i2 + 1 ];
            strncpy( options, &_options[ i2 ], i - i2 );
            options[ i - i2 ] = '\0';
        }
        else
             options = 0;
        // find read log options
        for( ++i, i2 = i; i < len && _options[ i ] != ';'; i++ );
        if( i > i2 )
        {
            readLogOptions = new char[ i - i2 + 1 ];
            strncpy( readLogOptions, &_options[ i2 ], i - i2 );
            readLogOptions[ i - i2 ] = '\0';
        }
        else
            readLogOptions = 0;
        // find write log options
        for( ++i, i2 = i; i < len && _options[ i ] != ';'; i++ );
        if( i > i2 )
        {
            writeLogOptions = new char[ i - i2 + 1 ];
            strncpy( writeLogOptions, &_options[ i2 ], i - i2 );
            writeLogOptions[ i - i2 ] = '\0';
        }
        else
            writeLogOptions = 0;
        // find whether to read from log
        if( ++i < len )
            readFromLog = atoi( &_options[ i ] );
        else
            readFromLog = false;

        /*fprintf( stderr,
            "Opening wrapper: device '%s', read log device '%s', write log device '%s', read from log '%s'\n",
            options ? options : "<none>", readLogOptions ? readLogOptions : "<none>",
            writeLogOptions ? writeLogOptions : "<none>", readFromLog ? "yes" : "no" );
       */
        
        this->open( );
    }
    
    
    // destructor
    virtual ~CharDeviceWrapper( )
    {
        this->close( );
        if( options )
            delete[] options;
        if( readLogOptions )
            delete[] readLogOptions;
        if( writeLogOptions )
            delete[] writeLogOptions;
    }
};


#endif // __CHARDEVICEWRAPPER_H__
