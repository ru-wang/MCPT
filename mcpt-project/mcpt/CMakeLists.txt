bottle_package()

bottle_subdir(NAME common)
bottle_subdir(NAME parser)
bottle_subdir(NAME renderer)

bottle_library(
  NAME misc
  HDRS misc.hpp
  DEPS @cheers
       @eigen
       @spdlog
)

bottle_binary(
  NAME mcpt_main
  SRCS mcpt_main.cc
  DEPS @cheers
       @eigen
       @openmp
       @spdlog
       /common
       /parser
       /renderer
       :misc
)

bottle_binary(
  NAME catch2_main
  SRCS catch2_main.cc
  DEPS @catch2
       /common/geometry:test
       /parser/obj_parser:test
       :misc
)

include(CTest)
include(Catch)
bottle_expand(TEST_BINARY :catch2_main)
catch_discover_tests(${TEST_BINARY})
