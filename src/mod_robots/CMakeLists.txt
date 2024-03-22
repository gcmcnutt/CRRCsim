set(MOD_ROBOTS_SRCS
  fdm_playback.cpp
  robot.cpp
  robotfile.cpp
  )
add_library(mod_robots ${MOD_ROBOTS_SRCS})

set (MOD_ROBOTS_LIBS    )
set (MOD_ROBOTS_INCDIRS )
    
include_directories(
  ${MOD_ROBOTS_EXT_INCDIRS}
)

link_directories ( ${MOD_ROBOTS_LINKDIRS} )
