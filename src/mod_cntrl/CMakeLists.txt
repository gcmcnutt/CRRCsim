set(MOD_CNTRL_SRCS
  cntrl_mcopter01/cntrl_mcopter01.cpp
  cntrl_scalethrottle/scalethrottle.cpp
  cntrl_limitflipthr/limitflipthrottle.cpp
  cntrl_initinputs/cntrl_initinputs.cpp
  cntrl_omega/cntrl_omega.cpp
  cntrl_inavfwrate/cntrl_inavfwrate.cpp
  cntrl_steptest/cntrl_steptest.cpp   # 043 T046 — responsiveness sanity test
  cntrl_phugoid/cntrl_phugoid.cpp
  cntrl_rateofclimb/cntrl_rateofclimb.cpp
  cntrl_setuserinput/cntrl_setuserinput.cpp
  controller.cpp
  )
add_library(mod_cntrl ${MOD_CNTRL_SRCS})

# 043 US2 — cntrl_inavfwrate.cpp includes the pure loop core
# include/autoc/control/inav_fw_rate.h (autoc-side, header-only).
target_include_directories(mod_cntrl PRIVATE ${CMAKE_SOURCE_DIR}/include)

set (MOD_CNTRL_LIBS    )
set (MOD_CNTRL_INCDIRS )

link_directories      ( ${MOD_CNTRL_LINKDIRS} )
