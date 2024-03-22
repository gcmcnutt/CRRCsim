set(MOD_CHARDEVICE_SRCS
 bufferedchardevice.cpp
 chardevicebase.cpp
 chardevicec.cpp
 chardevicewrapper.cpp
 filechardevice.cpp
 serialchardevice.cpp
 socketchardevicebase.cpp
 tcpchardevice.cpp
 tcpserverchardevice.cpp
 udpchardevice.cpp
 udpserverchardevice.cpp
  )
add_library(mod_chardevice ${MOD_CHARDEVICE_SRCS})

set (MOD_CHARDEVICE_LIBS    )
set (MOD_CHARDEVICE_INCDIRS )
    
include_directories(
  ${MOD_CHARDEVICE_EXT_INCDIRS}
)

link_directories      ( ${MOD_CHARDEVICE_LINKDIRS} )
