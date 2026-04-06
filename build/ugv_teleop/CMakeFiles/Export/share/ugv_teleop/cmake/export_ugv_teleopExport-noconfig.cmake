#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ugv_teleop::ugv_teleop_core" for configuration ""
set_property(TARGET ugv_teleop::ugv_teleop_core APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ugv_teleop::ugv_teleop_core PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libugv_teleop_core.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS ugv_teleop::ugv_teleop_core )
list(APPEND _IMPORT_CHECK_FILES_FOR_ugv_teleop::ugv_teleop_core "${_IMPORT_PREFIX}/lib/libugv_teleop_core.a" )

# Import target "ugv_teleop::joy_axis_selector_node" for configuration ""
set_property(TARGET ugv_teleop::joy_axis_selector_node APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ugv_teleop::joy_axis_selector_node PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/ugv_teleop/joy_axis_selector_node"
  )

list(APPEND _IMPORT_CHECK_TARGETS ugv_teleop::joy_axis_selector_node )
list(APPEND _IMPORT_CHECK_FILES_FOR_ugv_teleop::joy_axis_selector_node "${_IMPORT_PREFIX}/lib/ugv_teleop/joy_axis_selector_node" )

# Import target "ugv_teleop::joy_launcher_node" for configuration ""
set_property(TARGET ugv_teleop::joy_launcher_node APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ugv_teleop::joy_launcher_node PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/ugv_teleop/joy_launcher_node"
  )

list(APPEND _IMPORT_CHECK_TARGETS ugv_teleop::joy_launcher_node )
list(APPEND _IMPORT_CHECK_FILES_FOR_ugv_teleop::joy_launcher_node "${_IMPORT_PREFIX}/lib/ugv_teleop/joy_launcher_node" )

# Import target "ugv_teleop::keyboard_teleop_node" for configuration ""
set_property(TARGET ugv_teleop::keyboard_teleop_node APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ugv_teleop::keyboard_teleop_node PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/ugv_teleop/keyboard_teleop_node"
  )

list(APPEND _IMPORT_CHECK_TARGETS ugv_teleop::keyboard_teleop_node )
list(APPEND _IMPORT_CHECK_FILES_FOR_ugv_teleop::keyboard_teleop_node "${_IMPORT_PREFIX}/lib/ugv_teleop/keyboard_teleop_node" )

# Import target "ugv_teleop::keyboard_tty_teleop_node" for configuration ""
set_property(TARGET ugv_teleop::keyboard_tty_teleop_node APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ugv_teleop::keyboard_tty_teleop_node PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/ugv_teleop/keyboard_tty_teleop_node"
  )

list(APPEND _IMPORT_CHECK_TARGETS ugv_teleop::keyboard_tty_teleop_node )
list(APPEND _IMPORT_CHECK_FILES_FOR_ugv_teleop::keyboard_tty_teleop_node "${_IMPORT_PREFIX}/lib/ugv_teleop/keyboard_tty_teleop_node" )

# Import target "ugv_teleop::keyboard_gui_teleop_node" for configuration ""
set_property(TARGET ugv_teleop::keyboard_gui_teleop_node APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ugv_teleop::keyboard_gui_teleop_node PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/ugv_teleop/keyboard_gui_teleop_node"
  )

list(APPEND _IMPORT_CHECK_TARGETS ugv_teleop::keyboard_gui_teleop_node )
list(APPEND _IMPORT_CHECK_FILES_FOR_ugv_teleop::keyboard_gui_teleop_node "${_IMPORT_PREFIX}/lib/ugv_teleop/keyboard_gui_teleop_node" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
