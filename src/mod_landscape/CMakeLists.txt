set(MOD_LANDSCAPE_SRCS
  crrc_builtin_scenery.cpp
  crrc_scenery.cpp
  hd_ssgLOSterrain.cpp
  hd_tabulatedterrain.cpp
  hd_tilingterrain.cpp
  heightdata.cpp
  model_based_scenery.cpp
  wind_from_terrain.cpp
  winddata3D.cpp
  )
add_library(mod_landscape ${MOD_LANDSCAPE_SRCS})

set (MOD_LANDSCAPE_LIBS    )
set (MOD_LANDSCAPE_INCDIRS )
    
include_directories(
  ${MOD_LANDSCAPE_EXT_INCDIRS}
)

link_directories      ( ${MOD_LANDSCAPE_LINKDIRS} )
