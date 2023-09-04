cmake_minimum_required(VERSION 3.14)
project(mcpt-superproject LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "Setting build type to: ${CMAKE_BUILD_TYPE}")

find_program(CCACHE ccache)
if(CCACHE)
  message(STATUS "Found ccache: ${CCACHE}")
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ${CCACHE})
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ${CCACHE})
endif()

if(APPLE)
  set(EXTRA_COMPILE_OPTIONS -fcolor-diagnostics -pedantic -Wall)
else()
  set(EXTRA_COMPILE_OPTIONS -fdiagnostics-color=always -pedantic -Wall)
endif()

add_compile_options(${EXTRA_COMPILE_OPTIONS}
  $<$<CONFIG:Debug>:-O0> $<$<CONFIG:Debug>:-g>
  $<$<CONFIG:Release>:-O3> $<$<CONFIG:Release>:-DNDASSERT>)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} CACHE INTERNAL "")

enable_testing()
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/mcpt-project)
