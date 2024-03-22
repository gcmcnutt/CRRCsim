set(MOD_FDM_SRCS
  eom01/eom01.cpp
  fdm_002/fdm_002.cpp
  fdm_displaymode/fdm_displaymode.cpp
  gear01/gear.cpp
  fdm_heli01/fdm_heli01.cpp
  fdm_larcsim/fdm_larcsim.cpp
  fdm_mcopter01/fdm_mcopter01.cpp
  fdm_testmode/fdm_testmode.cpp
  formats/airtoxml.cpp
  physics/eom.cpp
  power/battery.cpp
  power/engine_dcm.cpp
  power/gearing.cpp
  power/power.cpp
  power/propeller.cpp
  power/shaft.cpp
  power/simplethrust.cpp
  fdm.cpp
  ls_geodesy.cpp
  xmlmodelfile.cpp
  )
add_library(mod_fdm ${MOD_FDM_SRCS})

set (MOD_FDM_LIBS    )
set (MOD_FDM_INCDIRS )
    
include_directories(
  ${MOD_FDM_EXT_INCDIRS}
)

link_directories ( ${MOD_FDM_LINKDIRS} )
