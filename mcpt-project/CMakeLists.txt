project(mcpt-core VERSION 0.1 LANGUAGES CXX)

if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.19")
  cmake_policy(SET CMP0110 NEW)
endif()

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE TRUE)

include(FetchContent)
include(com_github_ru-wang_bottle)

bottle_subdir(NAME depends)
bottle_subdir(NAME mcpt)
