set(MOD_MODE_SRCS
  F3A/handlerF3A.cpp
  F3F/handlerF3F.cpp
  )
add_library(mod_mode ${MOD_MODE_SRCS})

set (MOD_MODE_LIBS    )
set (MOD_MODE_INCDIRS )
    
include_directories(
  ${MOD_MODE_EXT_INCDIRS}
)

link_directories      ( ${MOD_MODE_LINKDIRS} )
