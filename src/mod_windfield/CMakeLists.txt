set(MOD_WINDFIELD_SRCS
  thermal03/solve.cpp
  thermal03/thermikschale.cpp
  thermal03/tschalen.cpp
  windfield.cpp
  )
add_library(mod_windfield ${MOD_WINDFIELD_SRCS})

set (MOD_WINDFIELD_LIBS    )
set (MOD_WINDFIELD_INCDIRS )

include_directories(
  ${MOD_WINDFIELD_EXT_INCDIRS}
)

link_directories      ( ${MOD_WINDFIELD_LINKDIRS} )
