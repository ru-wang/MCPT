bottle_package()

bottle_library(
  NAME fileserver
  SRCS fileserver.cpp
  HDRS fileserver.hpp
  DEPS @spdlog
       //mcpt/common:assert
)
