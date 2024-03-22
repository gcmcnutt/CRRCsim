set(MOD_VIDEO_SRCS
  airplane_vis.cpp
  crrc_animation.cpp
  crrc_graphics.cpp
  crrc_sky.cpp
  crrc_ssgutils.cpp
  fonts.cpp
  glconsole.cpp
  gloverlay.cpp
  ssgLoadJPG.cpp
  shadow_volume.cpp
  )
add_library(mod_video ${MOD_VIDEO_SRCS})
    
include_directories(
  ${MOD_VIDEO_EXT_INCDIRS}
)

link_directories      ( ${MOD_VIDEO_LINKDIRS} )
