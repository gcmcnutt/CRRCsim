set(MOD_MATH_SRCS
  intgr.cpp
  linearreg.cpp
  matrix33.cpp
  pt1.cpp
  quaternion.cpp
  vector3.cpp
  )
add_library(mod_math ${MOD_MATH_SRCS})

set (MOD_MATH_LIBS    )
set (MOD_MATH_INCDIRS )
    
link_directories      ( ${MOD_MATH_LINKDIRS} )

add_executable       (quat_test quat_test.cpp )
target_link_libraries(quat_test mod_math)

add_executable       (m44_test m44_test.cpp)
target_link_libraries(m44_test mod_math)
