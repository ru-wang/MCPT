bottle_package()

bottle_subdir(NAME obj_parser)

bottle_library(
  NAME misc
  SRCS misc.cpp
  HDRS misc.hpp
  DEPS @spdlog
       //mcpt/common:assert
)

bottle_library(
  NAME mtl_parser
  SRCS mtl_parser.cpp
  HDRS mtl_parser.hpp
  DEPS @spdlog
       //mcpt/common/object
       //mcpt/common:assert
       :misc
)
