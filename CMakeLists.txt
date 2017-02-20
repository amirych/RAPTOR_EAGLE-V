project(RAPTOR_EAGLE-V)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_BUILD_TYPE "Debug")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")
find_package(XCLIB QUIET)

if (XCLIB_FOUND)
    message(STATUS "XCLIB IS FOUND!")
    message(STATUS "XCLIB_INCLUDE_DIR: " ${XCLIB_INCLUDE_DIR})
    message(STATUS "XCLIB_LIBRARIES: " ${XCLIB_LIBRARIES})
endif()


set (EAGLE_CAMERA_VERSION_MAJOR 0)
set (EAGLE_CAMERA_VERSION_MINOR 1)

set (EAGLE_CAMERA_MAX_XBIN 64)
set (EAGLE_CAMERA_MAX_YBIN 64)

set (EAGLE_CAMERA_CCD_WIDTH 2048)
set (EAGLE_CAMERA_CCD_HEIGHT 2048)

configure_file (
  "camera/eagle_camera_config.h.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/camera/eagle_camera_config.h"
  #  "${PROJECT_BINARY_DIR}/eagle_camera_config.h"
  )


# API library
set(EAGLE_CAMERA_LIB eagle_camera)
include_directories(./camera ${XCLIB_INCLUDE_DIR})
aux_source_directory(./camera/ EAGLE_CAMERA_SRC)
add_library(${EAGLE_CAMERA_LIB} SHARED ${EAGLE_CAMERA_SRC})
target_link_libraries(${EAGLE_CAMERA_LIB} ${XCLIB_LIBRARIES})
target_compile_definitions(${EAGLE_CAMERA_LIB} PRIVATE EAGLE_CAMERA_LIBRARY=1)


# temporary test suite
set(TEST_PROG test_prog)
add_executable(${TEST_PROG} test_prog.cpp)
target_link_libraries(${TEST_PROG} ${EAGLE_CAMERA_LIB})

SET(CPACK_GENERATOR "TBZ2")
include(CPack)
