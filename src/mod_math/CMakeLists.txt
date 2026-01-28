set(MOD_MATH_SRCS
  intgr.cpp
  linearreg.cpp
  pt1.cpp
  quaternion.cpp
  )
add_library(mod_math ${MOD_MATH_SRCS})

# Eigen is header-only, linked at project level
target_link_libraries(mod_math Eigen3::Eigen)

set (MOD_MATH_LIBS    )
set (MOD_MATH_INCDIRS )

link_directories      ( ${MOD_MATH_LINKDIRS} )

add_executable       (quat_test quat_test.cpp )
target_link_libraries(quat_test mod_math Eigen3::Eigen)

# TODO: m44_test needs Matrix44 API updates for Eigen migration
# add_executable       (m44_test m44_test.cpp)
# target_link_libraries(m44_test mod_math Eigen3::Eigen)
