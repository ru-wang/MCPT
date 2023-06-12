bottle_package()

bottle_library(
  NAME context
  HDRS context.hpp
  DEPS @eigen
       //mcpt/common/object
)

bottle_library(
  NAME line_parser
  SRCS line_parser.cpp
  HDRS line_parser.hpp
  DEPS @eigen
       @spdlog
       //mcpt/common:assert
       //mcpt/parser:misc
       //mcpt/parser:mtl_parser
       :context
)

bottle_library(
  NAME parser
  SRCS parser.cpp
  HDRS parser.hpp
  DEPS //mcpt/common/object
       :context
       :line_parser
)

bottle_library(
  NAME line_parser_test
  SRCS line_parser_test.cpp
  DEPS @catch2
       @eigen
       @spdlog
       //mcpt/common/object
       :context
       :line_parser
  XCLD
)

bottle_library(
  NAME test
  DEPS :line_parser_test
  XCLD
)
