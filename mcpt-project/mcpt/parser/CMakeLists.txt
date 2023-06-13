bottle_package()

bottle_subdir(NAME mtl_parser)
bottle_subdir(NAME obj_parser)

bottle_library(
  NAME misc
  SRCS misc.cpp
  HDRS misc.hpp
  DEPS @spdlog
       //mcpt/common:assert
)
