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
  DEPS //mcpt/common:misc
       :context
       :tokenizer
)

bottle_library(
  NAME parser
  SRCS parser.cpp
  HDRS parser.hpp
  DEPS //mcpt/common/object
       //mcpt/common:assert
       :context
       :line_parser
)

bottle_library(
  NAME tokenizer
  SRCS tokenizer.cpp
  HDRS tokenizer.hpp
  DEPS //mcpt/common:assert
       //mcpt/parser/mtl_parser:parser
       :context
)

bottle_library(
  NAME tokenizer_test
  SRCS tokenizer_test.cpp
  DEPS @catch2
       @eigen
       :context
       :test_helper
       :tokenizer
  XCLD
)

bottle_library(
  NAME test_helper
  SRCS test_helper.cpp
  HDRS test_helper.hpp
       test_mock.hpp
  DEPS @catch2
       @eigen
       @spdlog
       //mcpt/common/object
       :context
  XCLD
)

bottle_library(
  NAME parser_test
  SRCS parser_test.cpp
  DEPS @catch2
       //mcpt/common/object
       :parser
  XCLD
)

bottle_library(
  NAME test
  DEPS :parser_test
       :tokenizer_test
  XCLD
)
