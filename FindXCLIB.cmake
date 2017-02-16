#
#  Try to find or setup by user EPIX XClib library
#
#  The module defines:
#       XCLIB_FOUND        - system has XCLIB library
#       XCLIB_LIBRARIES    - libraries 
#       XCLIB_INCLUDE_DIR  - path to header file
#

include(FindPackageHandleStandardArgs)

set(XCLIB_INSTALL_DIR "" CACHE STRING "XCLIB instalation path")
set(XCLIB_INSTALL_DIR_INTERNAL "" CACHE STRING "XCLIB instalation path")

if (NOT "${XCLIB_INSTALL_DIR}" STREQUAL "${XCLIB_INSTALL_DIR_INTERNAL}")
  unset(XCLIB_INSTALL_DIR)
  unset(XCLIB_INSTALL_DIR_INTERNAL)
  find_path(XCLIB_INCLUDE_DIR NAMES xcliball.h PATHS ${XCLIB_INSTALL_DIR})
  find_path(XCLIB_LIBRARY NAMES kxclib_x86_64 xclib_x86_64  libkxclib_x86_64.so libxclib_x86_64.so PATHS ${XCLIB_INSTALL_DIR})
else() # try to find in system default paths
    find_path (XCLIB_INCLUDE_DIR xcliball.h)
    find_library (XCLIB_LIBRARY NAMES kxclib_x86_64 xclib_x86_64)
endif()

find_package_handle_standard_args(XCLIB  DEFAULT_MSG
                                  XCLIB_LIBRARY XCLIB_INCLUDE_DIR)
mark_as_advanced(XCLIB_INCLUDE_DIR XCLIB_LIBRARY)
set(XCLIB_LIBRARIES ${XCLIB_LIBRARY})

