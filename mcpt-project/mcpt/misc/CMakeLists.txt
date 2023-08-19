bottle_package()

bottle_library(
  NAME dispatcher
  SRCS dispatcher.cpp
  HDRS dispatcher.hpp
  DEPS @eigen
       @spdlog
       //mcpt/common
       //mcpt/renderer
)
