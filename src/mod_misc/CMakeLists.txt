set(MOD_MISC_SRCS
  SimpleXMLTransfer.cpp
  crrc_rand.cpp
  filesystools.cpp
  lib_conversions.cpp
  scheduler.cpp
  )
add_library(mod_misc ${MOD_MISC_SRCS})

set (MOD_MISC_LIBS    )
set (MOD_MISC_INCDIRS )
    
link_directories      ( ${MOD_MISC_LINKDIRS} )
