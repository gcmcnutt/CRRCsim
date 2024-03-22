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
// chardevicewrapper.cpp
//
//    A "wrapper" chardevice that allows any one of the other chardevices to be used
//    transparently.
//
// authors
//
//    Todd R. Templeton <ttemplet@eecs.berkeley.edu>
//
//----------------------------------------------------------------------------------


#include "chardevice.h"
#include "chardevicecommon.h"


namespace {

    // create new device (returns NULL on error)
    CharDevice *new_char_device( char *options, bool wait = true )
    {    
        assert( options );
        
        int i;
        int len = strlen( options );
        CharDevice *charDevice = 0;
        
        // find IO type
        for( i = 0; i < len && options[ i ] != ','; i++ );
        assert( i < len );
        options[ i ] = '\0';
        if( strcmp( options, "serial" ) == 0 )
        {
            options[ i ] = ',';
            charDevice = new SerialCharDevice( &options[ i + 1 ], wait );
        }
        else if( strcmp( options, "tcp" ) == 0 )
        {
            options[ i ] = ',';
            charDevice = new TcpCharDevice( &options[ i + 1 ], wait );
        }
        else if( strcmp( options, "tcplisten" ) == 0 || strcmp( options, "tcpserver" ) == 0 )
        {
            options[ i ] = ',';
            charDevice = new TcpServerCharDevice( &options[ i + 1 ], wait );
        }
        else if( strcmp( options, "udp" ) == 0 )
        {
            options[ i ] = ',';
            charDevice = new UdpCharDevice( &options[ i + 1 ], wait );
        }
        else if( strcmp( options, "udplisten" ) == 0 || strcmp( options, "udpserver" ) == 0 )
        {
            options[ i ] = ',';
            charDevice = new UdpServerCharDevice( &options[ i + 1 ], wait );
        }
        else if( strcmp( options, "file" ) == 0 )
        {
            options[ i ] = ',';
            charDevice = new FileCharDevice( &options[ i + 1 ], wait );
        }
        else
        {
            assert( strcmp( options, "serial" ) == 0 || strcmp( options, "tcp" ) == 0 || strcmp( options, "tcplisten" ) == 0 || strcmp( options, "tcpserver" ) == 0 || strcmp( options, "udp" ) == 0 || strcmp( options, "udplisten" ) == 0 || strcmp( options, "udpserver" ) == 0 || strcmp( options, "file" ) == 0 );
            //options[ i ] = ',';
        }
        
        //charDevice->set_max_read_interval( maxInterval );
        
        return charDevice;
    }
    
} // namespace

// write to log
void CharDeviceWrapper::LogWriter::write_log( uint64_t time, const void *buf, size_t count )
{
    char sync[ 2 ] = { 0x55, 0x55 };
    uint32_t network_time = htonl( (uint32_t)( time / 1000 ) );
    uint16_t bc, network_bc;
    size_t b = 0, br = count;
    while( 1 )
    {
        bc = (uint16_t)br;
        network_bc = htons( bc );
        charDevice->write( (const void*)sync, 2 * sizeof(char) );
        charDevice->write( (const void*)( &network_time ), sizeof(uint32_t) );
        charDevice->write( (const void*)( &network_bc ), sizeof(uint16_t) );
        charDevice->write( (const void*)( &( (const char*)buf )[ b ] ), (size_t)bc );
        b += (size_t)bc;
        br -= (size_t)bc;
        if( br == 0 )
            break;
    }
}

// read from log
ssize_t CharDeviceWrapper::LogReader::read_log( uint64_t time, const void *buf, size_t count )
{
    ssize_t retval = 0;
    ssize_t bytesRead;
    while( 1 )
    {
        if( headerBytesRead < 8 )
        {
            headerBytesRead += charDevice->read( &header[ headerBytesRead ], 8 - headerBytesRead );
            if( headerBytesRead < 8 )
                break;
            assert( header[ 0 ] == 0x55 && header[ 1 ] == 0x55 );
            logItemTime = (uint64_t)( ntohl( *( (uint32_t*)( &header[ 2 ] ) ) ) ) * 1000;
            logItemBytes = ntohs( *( (uint16_t*)( &header[ 6 ] ) ) );
            logItemBytesRead = 0;
        }
        if( logItemTime <= time )
        {
            bytesRead = charDevice->read( &( ( (char*)buf )[ retval ] ), MIN( count - retval, (size_t)( logItemBytes - logItemBytesRead ) ) );
            logItemBytesRead += (uint16_t)bytesRead;
            retval += bytesRead;
            if( logItemBytesRead == logItemBytes )
            {
                headerBytesRead = 0;
                if( retval == (ssize_t)count )
                    break;
            }
            else
                break;
        }
        else
            break;
    }
    return retval;
}

// open device, called first time by constructor (uses wait, returns -1 on error)
int CharDeviceWrapper::open( void )
{
    if( readFromLog )
    {
        char *_readLogOptions = readLogOptions;
        bool allocedReadLogOptions = false;
        
        assert( readLogOptions );
        
        // in the special case of FileCharDevice, use write file as read (and only) file in readLogOptions
        if( strncmp( readLogOptions, "file,", 5 ) == 0 )
        {
            int i;
            int len = strlen( readLogOptions );
            
            // find output file
            for( i = 5; i < len && readLogOptions[ i ] != ','; i++ );
            i++;
            assert( i < len );
            
            _readLogOptions = new char[ 5 + ( len - i ) + 1 ];
            strcpy( _readLogOptions, "file," );
            strcpy( &_readLogOptions[ 5 ], &readLogOptions[ i ] );
            allocedReadLogOptions = true;
        }
        
        charDevice = 0;
        
        // always wait for connection/reconnection to logs
        readLogCharDevice = new_char_device( _readLogOptions, true );
        writeLogCharDevice = writeLogOptions ? new_char_device( writeLogOptions, true ) : 0;
    
        // log reader/writer
        logReader = new LogReader( readLogCharDevice );
        readLogWriter = 0;
        writeLogWriter = writeLogOptions ? new LogWriter( writeLogCharDevice ) : 0;
        
        if( allocedReadLogOptions )
            delete[] _readLogOptions;
    }
    else
    {
        assert( options );
    
        charDevice = new_char_device( options, wait );
        charDevice->set_max_read_interval( maxInterval );
        
        // always wait for connection/reconnection to logs
        readLogCharDevice = readLogOptions ? new_char_device( readLogOptions, true ) : 0;
        writeLogCharDevice = writeLogOptions ? new_char_device( writeLogOptions, true ) : 0;
        
        // log reader/writers
        logReader = 0;
        readLogWriter = readLogOptions ? new LogWriter( readLogCharDevice ) : 0;
        writeLogWriter = writeLogOptions ? new LogWriter( writeLogCharDevice ) : 0;
        loggingOn = ( readLogOptions || writeLogOptions );
    }
    
    return 0;
}

// read from device (buf = buffer into which to read, count = max number of bytes to read,
//   uses maxInterval and wait, returns number of bytes read or -1 on error)
ssize_t CharDeviceWrapper::read( void *buf, size_t count )
{
    ssize_t retval;
    if( readFromLog )
        retval = logReader->read_log( this->get_time( ), buf, count );
    else
    {
        retval = charDevice->read( buf, count );
        if( readLogWriter && loggingOn && retval > 0 )
            readLogWriter->write_log( this->get_time( ), buf, retval );
    }
    return retval;
}

// write to device (buf = buffer from which to write, count = number number of bytes to write,
//   uses wait, returns number of bytes written or -1 on error)
ssize_t CharDeviceWrapper::write( const void *buf, size_t count )
{
    if( writeLogWriter && loggingOn && count > 0 )
        writeLogWriter->write_log( this->get_time( ), buf, count );
    return readFromLog ? count : charDevice->write( buf, count );
}

// close device, called last time by destructor (returns 0 on success or -1 on error)
int CharDeviceWrapper::close( void )
{
    if( logReader )
        delete logReader;
    if( readLogWriter )
        delete readLogWriter;
    if( writeLogWriter )
        delete writeLogWriter;
    if( charDevice )
        delete charDevice;
    if( readLogCharDevice )
        delete readLogCharDevice;
    if( writeLogCharDevice )
        delete writeLogCharDevice;
    return 0;
}

// run script from file (returns -1 on error)
int CharDeviceWrapper::run_script( const char *scriptfile )
{
    char c;
    int sleepTime;
    char *buf;
    size_t nbytes, _nbytes;
    ssize_t written;
    FileCharDevice *script;
    int retval = 0;

    buf = new char[ 1024 ];
    script = new FileCharDevice( scriptfile, false );
    while( script->read( &c, 1 ) > 0 )
    {
        buf[ 0 ] = c;
        nbytes = 1;
        if( c == '_' )
        {
            while( nbytes < 9 && script->read( &c, 1 ) > 0 )
                buf[ nbytes++ ] = c;
            buf[ nbytes ] = '\0';
            if( strcmp( buf, "__SLEEP__" ) == 0 )
            {
                while( script->read( &c, 1 ) > 0 )
                {
                    buf[ nbytes++ ] = c;
                    if( c != ' ' && c != '\t' )
                        break;
                }
                _nbytes = nbytes - 1;
                while( script->read( &c, 1 ) > 0 )
                {
                    buf[ nbytes++ ] = c;
                    if( c == ' ' || c == '\t' || c == '\r' || c == '\n' )
                        break;
                }
                buf[ nbytes ] = '\0';
                if( nbytes > _nbytes && ( sleepTime = atoi( &buf[ _nbytes ] ) ) > 0 )
                {
                    while( script->read( &c, 1 ) > 0 && c != '\n' );
                    printf( "Run script: sleeping for %d ms\n", sleepTime );
                    fflush( stdout );
                    usleep( sleepTime * 1000 );
                    continue;
                }
            }
        }
        written = this->write( buf, nbytes );
        if( (size_t)written != nbytes )
            retval = -1;
    }

    delete script;
    delete[] buf;

    return retval;
}
