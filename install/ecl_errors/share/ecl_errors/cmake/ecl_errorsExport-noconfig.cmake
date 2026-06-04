#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ecl_errors::ecl_errors" for configuration ""
set_property(TARGET ecl_errors::ecl_errors APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ecl_errors::ecl_errors PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libecl_errors.so.1.2.0"
  IMPORTED_SONAME_NOCONFIG "libecl_errors.so.1.2.0"
  )

list(APPEND _cmake_import_check_targets ecl_errors::ecl_errors )
list(APPEND _cmake_import_check_files_for_ecl_errors::ecl_errors "${_IMPORT_PREFIX}/lib/libecl_errors.so.1.2.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
