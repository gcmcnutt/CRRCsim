set(MOD_INPUTDEV_SRCS
  inputdev_audio/inputdev_audio.cpp
  inputdev.cpp
  inputdev_mnav/inputdev_mnav.cpp
  inputdev_mnav/inputdev_mnav.h
  inputdev_mnav/mnav.cpp
  inputdev_parallel/inputdev_parallel.cpp
  inputdev_PPM/inputdev_PPM.cpp
  inputdev_rctran/inputdev_rctran.cpp
  inputdev_rctran2/inputdev_rctran2.cpp
  inputdev_serial/inputdev_serial.cpp
  inputdev_serial/inputdev_serial.h
  inputdev_serial2/inputdev_serial2.cpp
  inputdev_serial2/LoggerReader_byte.cpp
  inputdev_serial2/LoggerReader_ttyS.cpp
  inputdev_serpic/inputdev_serpic.cpp
  inputdev_software/inputdev_software.cpp
  inputdev_zhenhua/inputdev_zhenhua.cpp
  inputdev_ct6a/inputdev_ct6a.cpp
 )
add_library(mod_inputdev ${MOD_INPUTDEV_SRCS})

set (MOD_INPUTDEV_LIBS    )
set (MOD_INPUTDEV_INCDIRS )
    
include_directories(
  ${MOD_INPUTDEV_EXT_INCDIRS}
)

link_directories      ( ${MOD_INPUTDEV_LINKDIRS} )
