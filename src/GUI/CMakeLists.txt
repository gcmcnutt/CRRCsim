set(GUI_SRCS
 crrc_audio.cpp
 crrc_calibmap.cpp
 crrc_ctrldev.cpp
 crrc_ctrlgen.cpp
 crrc_dialog.cpp
 crrc_f3a.cpp
 crrc_f3f.cpp
 crrc_gui_main.cpp
 crrc_gui_viewwind.cpp
 crrc_joy.cpp
 crrc_launch.cpp
 crrc_loadrobot.cpp
 crrc_location.cpp
 crrc_mousebutton.cpp
 crrc_msgbox.cpp
 crrc_planesel.cpp
 crrc_scaleinput.cpp
 crrc_setrecordname.cpp
 crrc_slider.cpp
 crrc_spin.cpp
 crrc_video.cpp
 crrc_windthermal.cpp
 puaFileBox.cpp
 puaGLPreview.cpp
 puaScrListBox.cpp
 puaImageFrame.cpp
 util.cpp
  )
add_library(GUI ${GUI_SRCS})

set (GUI_LIBS    )
set (GUI_INCDIRS )
    
include_directories(
  ${GUI_EXT_INCDIRS}
)

link_directories      ( ${GUI_LINKDIRS} )
