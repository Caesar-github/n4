# ----------------------------------------------------------------------------
#
# hertz wangh@rock-chips.com
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# ----------------------------------------------------------------------------

# vi: set noexpandtab syntax=cmake:

project(rk_n4)

cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)

# ----------------------------------------------------------------------------
# Compiler detection
# ----------------------------------------------------------------------------
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set(CLANG true)
endif()
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  set(GCC true)
endif()

if($(CMAKE_BUILD_TYPE) MATCHES "Release")
  set(CMAKE_C_VISIBILITY_PRESET hidden)
  set(CMAKE_CXX_VISIBILITY_PRESET hidden)
endif()
find_package(PkgConfig)

# set c++11
set(CMAKE_CXX_STANDARD 11)
# rtti may be expensive on Embedded Platform, obey Google C++ Style Guide
add_definitions(-fno-rtti)

add_definitions(-fPIC)
add_definitions(-Werror)

pkg_check_modules(LIBSERIALPORT libserialport)
pkg_check_modules(LIBEASYMEDIA libeasymedia)
if(LIBEASYMEDIA_FOUND)
  add_executable(n4 main.cc)
  target_include_directories(n4
                             PUBLIC ${LIBSERIALPORT_INCLUDE_DIRS}
                             PUBLIC ${LIBEASYMEDIA_INCLUDE_DIRS})
  target_link_libraries(n4 serialport easymedia)
  install(TARGETS n4 RUNTIME DESTINATION "bin")
else()
  message(FATAL_ERROR "libeasymedia not found")
endif()
