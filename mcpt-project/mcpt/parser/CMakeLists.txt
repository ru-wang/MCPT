bottle_package()

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

bottle_library(
  NAME obj_parser
  SRCS obj_line_parser.cpp
       obj_parser.cpp
  HDRS obj_line_parser.hpp
       obj_parser.hpp
  DEPS @eigen
       //mcpt/common/object
       //mcpt/common:assert
       :misc
       :mtl_parser
)

bottle_library(
  NAME obj_parser_test
  SRCS obj_parser_test.cpp
  DEPS @catch2
       @eigen
       @spdlog
       //mcpt/common/object
       :obj_parser
)

bottle_library(
  NAME test
  DEPS :obj_parser_test
)
