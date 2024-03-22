dnl First attempt to create a GNU auto* tools scheme for CRRC
dnl Sep 1 2007
dnl Jan Reucker <slowhand_47@gmx.de>
dnl Andreas Rueckert <mail@andreas-rueckert.de> 

dnl Define package and identify version from hg
m4_define([tmp_package], [CRRCsim])
m4_define([tmp_tag], m4_esyscmd([hg parents --template {latesttag}]))
m4_define([tmp_id], m4_esyscmd([hg id --id] | tr -d '\n'))
m4_define([tmp_ver], [tmp_tag] ([tmp_id]))

dnl Init the autoconf macros
AC_INIT(crrcsim, 0.9.13, crrcsim-devel@lists.sourceforge.net)

dnl Init automake
AM_INIT_AUTOMAKE

dnl This project is written in C++
AC_LANG(C++)

dnl Make sure we're in the right directory
AC_CONFIG_SRCDIR([src/crrc_main.h])

dnl Output goes into this header file
AC_CONFIG_HEADER([crrc_config.h])

dnl Checks for programs.
AC_PROG_CXX

dnl Checks for header files.
AC_HEADER_STDC

dnl Host-specific checks
AC_CANONICAL_HOST

XTRA_OBJS=

case "${host}" in
*-*-mingw32*)
    dnl MinGW/MSYS under Windows.

    CXXFLAGS="$CXXFLAGS -DWIN32"
    dnl For some strange reason, /usr/include is not in the
    dnl standard include path, so we add it here.
    CPPFLAGS="$CPPFLAGS -I/usr/include"
    XTRA_OBJS="win32icon.res"

    dnl Additional libraries and linker switches. Again,
    dnl for some strange reason, /usr/lib is not in the
    dnl standard lib path, so we add it here.
    LIBS="$LIBS -lwinmm -lwsock32 -mwindows -lmingw32 -lintl"
    LDFLAGS="$LDFLAGS -L/usr/lib"

    ;;
*-apple-darwin*)
    dnl Mac OS X

    CXXFLAGS="$CXXFLAGS -DMACOSX"

    ;;
*)
    dnl other
    CXXFLAGS="$CXXFLAGS -Dlinux"
    dnl ACX_PTHREAD([PA_CHECK_LIBS=-lpthread], [])
    AC_CHECK_LIB(pthread, pthread_create,[PA_CHECK_LIBS=-lpthread]
                ,
                AC_MSG_ERROR([libpthread not found!]))
    ;;
esac

AC_SUBST([XTRA_OBJS])

dnl enable compiler warnings
CXXFLAGS="$CXXFLAGS -Wall"

dnl For Linux and the parallel interface, check if we need <sys/io.h>
dnl or <asm/io.h>. This replaces the old LINUX_LIBC5 macro.
AC_CHECK_HEADERS([sys/io.h asm/io.h])

dnl check for required libs

dnl Search for a portaudio lib

dnl Default is: no portaudio present
portaudio=0

dnl Check for the Portaudio header file
AC_CHECK_HEADER([portaudio.h])

if  (test "x$ac_cv_header_portaudio_h" = "xyes"); then 
  dnl Check for Portaudio 18
  AC_CHECK_LIB(portaudio,Pa_CountDevices, [portaudio=18], [], ${PA_CHECK_LIBS})

  dnl Check for Portaudio 19
  AC_CHECK_LIB(portaudio, Pa_GetDeviceCount, [portaudio=19], [], ${PA_CHECK_LIBS})

  dnl Show the result
  if test $portaudio == 18
  then
    dnl AC_MSG_NOTICE([Portaudio 18 found])
    AC_DEFINE([PORTAUDIO], [18], [Portaudio version, 0 to disable audio interface])
    PA_LIBS=-lportaudio
    has_portaudio="yes  (found Portaudio V18)"
  else if test $portaudio == 19
    then
      dnl AC_MSG_NOTICE([Portaudio 19 found])
      AC_DEFINE([PORTAUDIO], [19], [Portaudio version, 0 to disable audio interface])
      PA_LIBS=-lportaudio
      has_portaudio="yes  (found Portaudio V19)"
    else
      dnl AC_MSG_NOTICE([No Portaudio found])
      AC_DEFINE([PORTAUDIO], [0], [Portaudio version, 0 to disable audio interface])
      has_portaudio="no   (Portaudio library not found)"
      PA_LIBS=
    fi
  fi
else
  AC_DEFINE([PORTAUDIO], [0], [Portaudio version, 0 to disable audio interface])
  has_portaudio="no   (Portaudio header not found)"
  PA_LIBS=
fi
AC_SUBST(PA_LIBS)

dnl Search for SDL 1.2.5 or newer
SDL_VERSION=1.2.5
sdl_mousewheel="yes  (SDL >= 1.2.5)"
AM_PATH_SDL($SDL_VERSION,:,AC_MSG_NOTICE([*** SDL version $SDL_VERSION or newer not found! See if at least 1.2.0 is present...]))
if test "x$no_sdl" = "xyes"
then
  dnl If 1.2.5 or newer was not found, try to find at least SDL >= 1.2.0.
  dnl In this case we'll have to disable mouse wheel support.
  SDL_VERSION=1.2.0
  AM_PATH_SDL($SDL_VERSION,:,AC_MSG_ERROR([*** SDL version $SDL_VERSION or newer not found!]))
  AC_DEFINE([SDL_WITHOUT_MOUSEWHEEL], [1], [SDL < 1.2.5 did not feature mousewheel button macros])
  sdl_mousewheel="no   (SDL < 1.2.5)"
fi
AC_SUBST(SDL_LIBS)
AC_SUBST(SDL_CFLAGS)

dnl Search for OpenGL and GLU
AX_CHECK_GLU

dnl The following checks lead to endless recursion on errors...
dnl if (test "no_gl" = "yes")
dnl then
dnl   echo "OpenGL libs and/or headers not found. You *must* have OpenGL"
dnl   echo "installed on your system to build CRRCsim!"
dnl   echo
dnl   echo "configure aborted."
dnl   exit
dnl fi
dnl if (test "no_glu" = "yes")
dnl then
dnl   echo "GLU not found. You *must* have GLU installed on your system to build CRRCsim!"
dnl   echo
dnl   echo "configure aborted."
dnl   exit
dnl fi

dnl Search for PLIB 1.8.4
dnl PLIB consist of several more or less independent
dnl components, so we better check each component we need.
AC_CHECK_HEADER([plib/ul.h])
AC_CHECK_HEADER([plib/sg.h])
AC_CHECK_HEADER([plib/ssg.h])
AC_CHECK_HEADER([plib/ssgaSky.h])
AC_CHECK_HEADER([plib/pu.h], [], [],
[#define PU_USE_NONE
])
AC_CHECK_HEADER([plib/puAux.h], [], [],
[#define PU_USE_NONE
])
if  (test "x$ac_cv_header_plib_puAux_h" != "xyes")    \
 || (test "x$ac_cv_header_plib_pu_h" != "xyes")       \
 || (test "x$ac_cv_header_plib_ssgaSky_h" != "xyes")  \
 || (test "x$ac_cv_header_plib_ssg_h" != "xyes")      \
 || (test "x$ac_cv_header_plib_sg_h" != "xyes")       \
 || (test "x$ac_cv_header_plib_ul_h" != "xyes"); then
    echo
    echo "You *must* have the plib library installed on your system to build CRRCsim!"
    echo
    echo "configure aborted."
    exit
fi

dnl Thanks to the FlightGear guys for this piece of code... 
AC_MSG_CHECKING([for plib 1.8.4 or newer])
AC_TRY_RUN([
#include <plib/ul.h>

#define MIN_PLIB_VERSION 184

int main() {
    if ( PLIB_VERSION < MIN_PLIB_VERSION ) {
         return -1;
    }

    return 0;
}
],
  AC_MSG_RESULT(yes),
  [AC_MSG_RESULT(wrong version);
   AC_MSG_ERROR([Install plib 1.8.4 or later first...])],
  AC_MSG_RESULT(yes)
)

dnl Check for libjpeg
AC_CHECK_HEADER(jpeglib.h)
if test "x$ac_cv_header_jpeglib_h" != "xyes"; then
    echo 
    echo "You *must* have libjpeg installed on your system to build CRRCsim!"
    echo
    echo "configure aborted."
    exit
fi

dnl Check for CGAL
AC_CHECK_HEADER(CGAL/Exact_predicates_inexact_constructions_kernel.h)
AC_CHECK_HEADER(CGAL/Delaunay_triangulation_3.h)
AC_CHECK_HEADER(CGAL/Triangulation_vertex_base_with_info_3.h)
AC_CHECK_HEADER(CGAL/assertions_behaviour.h)
if  (test "x$ac_cv_header_CGAL_Exact_predicates_inexact_constructions_kernel_h" = "xyes")    \
 && (test "x$ac_cv_header_CGAL_Delaunay_triangulation_3_h" = "xyes")       \
 && (test "x$ac_cv_header_CGAL_Triangulation_vertex_base_with_info_3_h" = "xyes"); then
    if  (test "x$ac_cv_header_CGAL_assertions_behaviour_h" = "xyes") then
      AC_DEFINE([CGAL_VERSION3], [0], [CGAL_VERSION3 compatibility]) 
      has_CGAL="yes  (found CGAL > v3)"
      AC_CHECK_LIB(gmp,__gmpq_init,,AC_MSG_ERROR([gmp library not found. Please install it.]))
      AC_CHECK_LIB(boost_thread-mt,main,,AC_MSG_ERROR([boost_thread-mt library not found. Please install it.]))
    else
      AC_DEFINE([CGAL_VERSION3], [1], [CGAL_VERSION3 compatibility])
      has_CGAL="yes  (found CGAL v3)"
    fi   
    CGAL_CFLAGS=-frounding-math
    CGAL_LIBS=-lCGAL
    AC_DEFINE([WINDDATA3D], [1], [Import code for wind data, needs CGAL, 0 to disable])
else
    has_CGAL="no   (CGAL not found)"
    CGAL_LIBS=
    CGAL_CFLAGS=
    AC_DEFINE([WINDDATA3D], [0], [Import code for wind data, needs CGAL, 0 to disable])
fi
AC_SUBST(CGAL_CFLAGS)
AC_SUBST(CGAL_LIBS)

AC_CONFIG_FILES([Makefile
                 documentation/Makefile
                 documentation/man/Makefile
                 documentation/models/Makefile
                 documentation/file_format/Makefile
                 documentation/power_propulsion/Makefile
                 documentation/howto_create_models/Makefile
                 documentation/input_method/Makefile
                 documentation/input_method/MNAV/Makefile
                 documentation/input_method/PARALLEL_1_to_3/Makefile
                 documentation/input_method/SERIAL2/Makefile
                 documentation/input_method/RCTRAN/Makefile
                 documentation/input_method/CT6A/Makefile
                 documentation/record_playback/Makefile
                 documentation/thermals/Makefile
                 locale/Makefile
                 models/Makefile
                 models/engine/Makefile
                 models/battery/Makefile
                 objects/Makefile
                 packages/Makefile
                 packages/Fedora/Makefile
                 packages/icons/Makefile
                 packages/Win32/Makefile
                 scenery/Makefile
                 sounds/Makefile
                 sounds/f3f/Makefile
                 sounds/f3f/default/Makefile
                 sounds/f3f/sport/Makefile
                 textures/Makefile])

AC_OUTPUT

echo
echo "Build options:"
echo
echo "    Mousewheel support: $sdl_mousewheel"
echo "    Audio interface:    $has_portaudio"
echo "    Wind data import:   $has_CGAL"
echo

if test $portaudio == 19
then
  echo
  echo "Attention:"
  echo
  echo "  Using Portaudio V19 is not recommended by the CRRCsim team. If"
  echo "  you experience frequent crashes when using the audio"
  echo "  interface, please install Portaudio V18 and recompile CRRCsim."
fi
