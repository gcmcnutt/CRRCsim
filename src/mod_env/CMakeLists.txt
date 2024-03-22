set(MOD_ENV_SRCS
  earth/atmos_62.cpp
  earth/ls_gravity.cpp
  )
add_library(mod_env ${MOD_ENV_SRCS})

set (MOD_ENV_LIBS    )
set (MOD_ENV_INCDIRS )
    
link_directories      ( ${MOD_ENV_LINKDIRS} )
