bottle_package()

bottle_library(
  NAME context
  HDRS context.hpp
  DEPS //mcpt/common/object
)

bottle_library(
  NAME parser
  SRCS parser.cpp
  HDRS parser.hpp
  DEPS //mcpt/common/object
       //mcpt/common:assert
       //mcpt/common:misc
       :context
       :tokenizer
)

bottle_library(
  NAME tokenizer
  SRCS tokenizer.cpp
  HDRS tokenizer.hpp
  DEPS //mcpt/common:assert
       :context
)
