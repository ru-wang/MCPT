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
  DEPS @spdlog
       //mcpt/parser:misc
       :context
       :tokenizer
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
  NAME tokenizer
  SRCS tokenizer.cpp
  HDRS tokenizer.hpp
  DEPS //mcpt/common:assert
       //mcpt/parser:mtl_parser
       :context
)

bottle_library(
  NAME tokenizer_test
  SRCS tokenizer_test.cpp
  DEPS @catch2
       @eigen
       @spdlog
       //mcpt/common/object
       :context
       :tokenizer
  XCLD
)

bottle_library(
  NAME test
  DEPS :tokenizer_test
  XCLD
)
