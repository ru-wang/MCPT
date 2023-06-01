FetchContent_Declare(com.github.ru-wang.bottle
  GIT_REPOSITORY https://github.com/ru-wang/bottle.git
  GIT_TAG        v0.1.2
  GIT_SHALLOW    TRUE)
FetchContent_MakeAvailable(com.github.ru-wang.bottle)

list(APPEND CMAKE_MODULE_PATH ${com.github.ru-wang.bottle_SOURCE_DIR})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} CACHE INTERNAL "")

include(bottle_target)
