#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "kobuki_random_walker::kobuki_random_walker" for configuration ""
set_property(TARGET kobuki_random_walker::kobuki_random_walker APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(kobuki_random_walker::kobuki_random_walker PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libkobuki_random_walker.so"
  IMPORTED_SONAME_NOCONFIG "libkobuki_random_walker.so"
  )

list(APPEND _cmake_import_check_targets kobuki_random_walker::kobuki_random_walker )
list(APPEND _cmake_import_check_files_for_kobuki_random_walker::kobuki_random_walker "${_IMPORT_PREFIX}/lib/libkobuki_random_walker.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
