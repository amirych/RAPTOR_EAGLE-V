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

configure_file (
  "camera/eagle_camera_config.h.in"
  "${PROJECT_BINARY_DIR}/eagle_camera_config.h"
  )

set(EAGLE_CAMERA_LIB eagle_camera)
include_directories(./camera ${XCLIB_INCLUDE_DIR})
aux_source_directory(./camera/ EAGLE_CAMERA_SRC)
add_library(${EAGLE_CAMERA_LIB} SHARED ${EAGLE_CAMERA_SRC})
target_link_libraries(${EAGLE_CAMERA_LIB} ${XCLIB_LIBRARIES})
