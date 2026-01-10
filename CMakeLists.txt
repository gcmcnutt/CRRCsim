# This is just a first try of a CMakeLists.txt. It will certainly not 
# make CRRCSim build on much more than a standard linux installation yet.
# It is only meant for dev purposes. Please look at the documentation
# on how to compile/install CRRCSim and so on.
#
# There is a script called cmake.sh for your convenience.


cmake_minimum_required(VERSION 2.4.6)
cmake_policy(VERSION 2.6)

project (crrcsim)

set(CMAKE_PACKAGE_NAME    "${PROJECT_NAME}")
set(CMAKE_PACKAGE_VERSION "0.9.13")

INCLUDE (CheckIncludeFiles)
INCLUDE (CheckIncludeFileCXX)
INCLUDE (CheckSymbolExists)
INCLUDE (CheckLibraryExists)
INCLUDE (CheckCXXSourceCompiles)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system serialization)

# Add GP_BUILD definition for autoc integration
add_definitions(-DGP_BUILD)

option(ENABLE_PROFILING "Build with frame pointers and debug info for perf/FlameGraph" OFF)
option(PERFORMANCE_BUILD "Build with high performance flags (keeps debug symbols for core dumps)" ON)

# uncomment to get verbose output
#set (CMAKE_VERBOSE_MAKEFILE ON)

#
# Host-specific checks
#
if (WIN32)
  add_definitions(-DWIN32)
else (WIN32)
  if (APPLE)
    # Mac OS X
    add_definitions(-DMACOSX -DCRRC_DATA_PATH="${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}")
  else (APPLE)
    add_definitions(-Dlinux -DCRRC_DATA_PATH="${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}")
  endif (APPLE)
endif (WIN32)



#
# enable compiler warnings
#
add_definitions(-Wall)

#
#
#
CHECK_INCLUDE_FILES("sys/io.h" HAVE_SYS_IO_H)

#
# Search for a portaudio lib
#
CHECK_INCLUDE_FILES ("portaudio.h" USE_PORTAUDIO)
if (USE_PORTAUDIO)
  check_library_exists(portaudio Pa_CountDevices   "${PORTAUDIO18_LIBRARY_DIR}" HAVE_PORTAUDIO18)
  
  if (HAVE_PORTAUDIO18)
    set(PORTAUDIO_LIBRARIES portaudio)    
    set(PORTAUDIO 18)
  else (HAVE_PORTAUDIO18)
    check_library_exists(portaudio Pa_GetDeviceCount "${PORTAUDIO19_LIBRARY_DIR}" HAVE_PORTAUDIO19)
    if (HAVE_PORTAUDIO19)
      set(PORTAUDIO_LIBRARIES portaudio)    
      set(PORTAUDIO 19)
    else (HAVE_PORTAUDIO19)    
      check_library_exists(portaudio2 Pa_GetDeviceCount "${PORTAUDIO19_LIBRARY_DIR}" HAVE_PORTAUDIO19)
      if (HAVE_PORTAUDIO19)
        set(PORTAUDIO_LIBRARIES portaudio2)
        set(PORTAUDIO 19)
      endif (HAVE_PORTAUDIO19)
    endif (HAVE_PORTAUDIO19)    
  endif (HAVE_PORTAUDIO18)    
else (USE_PORTAUDIO)
  set(PORTAUDIO 0)
endif (USE_PORTAUDIO)

#
# Search for SDL
#
INCLUDE (FindSDL)
if (SDL_FOUND)
  CHECK_SYMBOL_EXISTS(SDL_BUTTON_WHEELUP ${SDL_INCLUDE_DIR}/SDL.h HAS_SDL_MOUSEWHEEL)
  if (NOT HAS_SDL_MOUSEWHEEL)
    set(SDL_WITHOUT_MOUSEWHEEL 1)
  endif (NOT HAS_SDL_MOUSEWHEEL)
else (SDL_FOUND)
  message(FATAL_ERROR "SDL not found, but it is needed to compile ${PROJECT_NAME}")
endif (SDL_FOUND)

#
# Search for OpenGL and GLU
#
INCLUDE (FindOpenGL)
if (NOT OPENGL_FOUND OR NOT OPENGL_GLU_FOUND)
  message(FATAL_ERROR "OpenGL and GLU not found, but are needed to compile ${PROJECT_NAME}")
endif (NOT OPENGL_FOUND OR NOT OPENGL_GLU_FOUND)

#
# Search for PLIB 1.8.4
#
# PLIB consist of several more or less independent
# components, so we better check each component we need.
# Checking via "CHECK_INCLUDE_FILES" fails because gcc is used instead of g++. 
# How to change this?
#SET(PLIB_CHECK_INCLUDE
#    "plib/ul.h"     
#    "plib/sg.h"     
#    "plib/ssg.h"    
#    "plib/ssgaSky.h"
#    "plib/pu.h"     
#    "plib/puAux.h")
#CHECK_INCLUDE_FILES (${PLIB_CHECK_INCLUDE}  USE_PLIB)
# As the above doesn't work I use a program which determines 
# PLIB is >= 1.8.4, which is needed anyway.
#CHECK_CXX_SOURCE_COMPILES("#include <plib/ul.h>
#                           #include <plib/sg.h>
#                           #include <plib/ssg.h>
#                           #include <plib/ssgaSky.h>
#                           #include <plib/pu.h>
#                           #include <plib/puAux.h>
#                           int main(void)
#                           {
#                               return 0;
#                           }
#                          " USE_PLIB)
#if (NOT USE_PLIB)
#  message(FATAL_ERROR "PLIB not found, but is needed to compile ${PROJECT_NAME}")  
#endif (NOT USE_PLIB)
TRY_RUN(TEST_RETURNCODE TEST_DID_COMPILE "${CMAKE_BINARY_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake/test_plib.cpp")
if (TEST_DID_COMPILE AND TEST_RETURNCODE EQUAL 42)
  set(PLIB_LIBRARIES plibssg plibssgaux plibsg plibpuaux plibpu plibul plibfnt)
else (TEST_DID_COMPILE AND TEST_RETURNCODE EQUAL 42)
  message(FATAL_ERROR "PLIB >= 1.8.4 not found, but is needed to compile ${PROJECT_NAME}")
endif (TEST_DID_COMPILE AND TEST_RETURNCODE EQUAL 42)


#
# Check for libjpeg
#
find_package(JPEG)

#
# Check for CGAL
#
SET(CMAKE_REQUIRED_LIBRARIES CGAL)
CHECK_CXX_SOURCE_COMPILES(" #include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
                            #include <CGAL/Delaunay_triangulation_3.h>
                            #include <CGAL/Triangulation_vertex_base_with_info_3.h>
                            int main(void)
                            {
                              return 0;
                            }
                          " HAS_CGAL)
if (HAS_CGAL)
  set(CGAL_LIBRARIES CGAL)
  add_definitions(-frounding-math)
  CHECK_INCLUDE_FILE_CXX ("CGAL/assertions_behaviour.h" CGAL_IS_V3)
  if (NOT CGAL_IS_V3)
    set(CGAL_IS_V3 0)
    set(CGAL_MESSAGE "yes  (found CGAL > v3)")
  else (NOT CGAL_IS_V3)
    set(CGAL_MESSAGE "yes  (found CGAL v3)")
  endif (NOT CGAL_IS_V3)
else (HAS_CGAL)
  set(HAS_CGAL 0)
  set(CGAL_MESSAGE "no   (CGAL not found)")
endif (HAS_CGAL)



#
#
# set variables...
#
#

set(CMAKE_CXX_FLAGS_DEBUG "-g3 -O0 -Wno-write-strings")
set(CMAKE_EXE_LINKER_FLAGS "-no-pie")

if(ENABLE_PROFILING)
  message(STATUS "Enabling profiling-friendly compiler flags for crrcsim")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O2 -fno-omit-frame-pointer -fno-optimize-sibling-calls")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O2 -fno-omit-frame-pointer -fno-optimize-sibling-calls")
endif()

if(PERFORMANCE_BUILD)
  message(STATUS "Enabling performance-optimized build for crrcsim (with debug symbols)")
  # -O3: Aggressive optimization
  # -g: Debug symbols for core dump analysis (doesn't affect runtime)
  # -march=native: Use AVX-512/AVX2 and other CPU-specific instructions
  # -mtune=native: Optimize instruction scheduling for this CPU
  # -ffast-math: Allow FP optimizations (slightly non-IEEE compliant)
  # -funroll-loops: Better SIMD utilization
  # -flto: Link-time optimization for whole-program analysis
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -g -march=native -mtune=native -ffast-math -funroll-loops -flto")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -march=native -mtune=native -ffast-math -funroll-loops -flto")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -flto")
endif()

set(CRRCSIM_INCDIRS ${SDL_INCLUDE_DIR}
                    ${CMAKE_BINARY_DIR}
                    ${HAVE_SYS_IO_H}
   )                    
set(GUI_EXT_INCDIRS            ${CRRCSIM_INCDIRS})
set(MOD_CHARDEVICE_EXT_INCDIRS ${CRRCSIM_INCDIRS})
set(MOD_FDM_EXT_INCDIRS        ${CRRCSIM_INCDIRS})
set(MOD_INPUTDEV_EXT_INCDIRS   ${CRRCSIM_INCDIRS})
set(MOD_LANDSCAPE_EXT_INCDIRS  ${CRRCSIM_INCDIRS})
set(MOD_MAIN_EXT_INCDIRS       ${CRRCSIM_INCDIRS})
set(MOD_MODE_EXT_INCDIRS       ${CRRCSIM_INCDIRS})
set(MOD_WINDFIELD_EXT_INCDIRS  ${CRRCSIM_INCDIRS})
set(MOD_VIDEO_EXT_INCDIRS      ${CRRCSIM_INCDIRS})

set(CRRCSIM_SRCS
 src/aircraft.cpp
 src/config.cpp
 src/crrc_fdm.cpp
 src/crrc_keyboard.cpp
 src/crrc_loadair.cpp
 src/crrc_main.cpp
 src/crrc_sound.cpp
 src/crrc_soundserver.cpp
 src/crrc_system.cpp
 src/CTime.cpp
 src/global.cpp
 src/ImageLoaderTGA.cpp
 src/mouse_kbd.cpp
 src/record.cpp
 src/robots.cpp
 src/SimStateHandler.cpp
 src/zoom.cpp
  )

add_subdirectory(src/mod_chardevice)
add_subdirectory(src/GUI)
add_subdirectory(src/mod_cntrl)
add_subdirectory(src/mod_env)
add_subdirectory(src/mod_fdm)
add_subdirectory(src/mod_inputdev)
add_subdirectory(src/mod_landscape)
add_subdirectory(src/mod_main)
add_subdirectory(src/mod_math)
add_subdirectory(src/mod_misc)
add_subdirectory(src/mod_mode)
add_subdirectory(src/mod_robots)
add_subdirectory(src/mod_video)
add_subdirectory(src/mod_windfield)

set (CRRCSIM_LIBS
  GUI
  mod_cntrl
  mod_env
  mod_fdm
  mod_inputdev
  mod_video
  mod_landscape  
  mod_main
  mod_math
  mod_misc
  mod_mode
  mod_robots
  mod_windfield
  mod_chardevice
  ${SDL_LIBRARY}
  ${OPENGL_LIBRARIES}
  ${PORTAUDIO_LIBRARIES}
  ${CGAL_LIBRARIES}
  ${JPEG_LIBRARIES}
  ${PLIB_LIBRARIES}
  )

include_directories(${CRRCSIM_INCDIRS}
  ${EIGEN3_INCLUDE_DIR}
  $ENV{HOME}/GP/autoc
  $ENV{HOME}/GP/include
)

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/cmake/config.h.in ${CMAKE_BINARY_DIR}/crrc_config.h)

add_executable (crrcsim WIN32 ${CRRCSIM_SRCS})

option(USE_SANITIZER "Build with Address Sanitizer" OFF)
if(USE_SANITIZER)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=address -fno-omit-frame-pointer")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address")
endif()

target_link_directories(crrcsim PUBLIC $ENV{HOME}/GP/lib)

target_link_libraries (crrcsim ${CRRCSIM_LIBS} gp Eigen3::Eigen ${Boost_LIBRARIES})

message("")
message("Build options:")
message("")
message("    Mousewheel support: "${HAS_SDL_MOUSEWHEEL})
message("    Audio interface:    "${PORTAUDIO})
message("    Wind data import:   "${CGAL_MESSAGE})
message("")


if (PORTAUDIO EQUAL 19)
  message("")
  message("Attention:")
  message("")
  message("  Using Portaudio V19 is not recommended by the CRRCsim team. If")
  message("  you experience frequent crashes when using the audio")
  message("  interface, please install Portaudio V18 and recompile CRRCsim.")
  message("")
endif (PORTAUDIO EQUAL 19)

INSTALL(TARGETS crrcsim
        RUNTIME DESTINATION bin)
        
INSTALL(DIRECTORY models/        DESTINATION share/${PROJECT_NAME}/models)
INSTALL(DIRECTORY objects/       DESTINATION share/${PROJECT_NAME}/objects)
INSTALL(DIRECTORY scenery/       DESTINATION share/${PROJECT_NAME}/scenery)
INSTALL(DIRECTORY sounds/        DESTINATION share/${PROJECT_NAME}/sounds)
INSTALL(DIRECTORY textures/      DESTINATION share/${PROJECT_NAME}/textures)
INSTALL(DIRECTORY documentation/ DESTINATION share/doc/${PROJECT_NAME}
        PATTERN "Makefile"    EXCLUDE
        PATTERN "Makefile.in" EXCLUDE
        PATTERN "Makefile.am" EXCLUDE)
