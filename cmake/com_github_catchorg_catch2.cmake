set(CATCH_BUILD_TESTING OFF CACHE INTERNAL "")
set(CATCH_INSTALL_DOCS OFF CACHE INTERNAL "")
set(CATCH_INSTALL_HELPERS OFF CACHE INTERNAL "")
FetchContent_Declare(com.github.catchorg.Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v2.x
    GIT_SHALLOW    TRUE)
FetchContent_MakeAvailable(com.github.catchorg.Catch2)

list(APPEND CMAKE_MODULE_PATH ${com.github.catchorg.catch2_SOURCE_DIR}/contrib)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} CACHE INTERNAL "")

bottle_library(
  NAME catch2
  DEPS Catch2::Catch2
  DEFS CATCH_CONFIG_CONSOLE_WIDTH=120
)
