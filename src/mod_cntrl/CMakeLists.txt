set(MOD_CNTRL_SRCS
  cntrl_mcopter01/cntrl_mcopter01.cpp
  cntrl_scalethrottle/scalethrottle.cpp
  cntrl_limitflipthr/limitflipthrottle.cpp
  cntrl_initinputs/cntrl_initinputs.cpp
  cntrl_omega/cntrl_omega.cpp
  cntrl_phugoid/cntrl_phugoid.cpp
  cntrl_rateofclimb/cntrl_rateofclimb.cpp
  cntrl_setuserinput/cntrl_setuserinput.cpp
  controller.cpp
  )
add_library(mod_cntrl ${MOD_CNTRL_SRCS})

set (MOD_CNTRL_LIBS    )
set (MOD_CNTRL_INCDIRS )
    
link_directories      ( ${MOD_CNTRL_LINKDIRS} )
