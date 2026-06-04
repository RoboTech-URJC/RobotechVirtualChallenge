#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "kobuki_safety_controller::kobuki_safety_controller" for configuration ""
set_property(TARGET kobuki_safety_controller::kobuki_safety_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(kobuki_safety_controller::kobuki_safety_controller PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libkobuki_safety_controller.so"
  IMPORTED_SONAME_NOCONFIG "libkobuki_safety_controller.so"
  )

list(APPEND _cmake_import_check_targets kobuki_safety_controller::kobuki_safety_controller )
list(APPEND _cmake_import_check_files_for_kobuki_safety_controller::kobuki_safety_controller "${_IMPORT_PREFIX}/lib/libkobuki_safety_controller.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
