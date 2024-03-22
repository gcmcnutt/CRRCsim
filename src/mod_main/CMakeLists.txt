set(MOD_MAIN_SRCS
  eventhandler.cpp
  crrc_checkopts.cpp
  Event.cpp
  EventDispatcher.cpp
  )
add_library(mod_main ${MOD_MAIN_SRCS})

set (MOD_MAIN_LIBS    )
set (MOD_MAIN_INCDIRS )
    
include_directories(
  ${MOD_MAIN_EXT_INCDIRS}
)

link_directories      ( ${MOD_MAIN_LINKDIRS} )
