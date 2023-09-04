bottle_package()

bottle_subdir(NAME common)
bottle_subdir(NAME misc)
bottle_subdir(NAME parser)
bottle_subdir(NAME renderer)

bottle_binary(
  NAME mcpt_main
  SRCS mcpt_main.cc
  DEPS @eigen
       @spdlog
       /common
       /misc
       /parser
       /renderer
)

bottle_binary(
  NAME random_main
  SRCS random_main.cc
  DEPS @eigen
       /common
       /misc:logging
       /misc:visualizing
)

bottle_binary(
  NAME catch2_main
  SRCS catch2_main.cc
  DEPS @catch2
       /common/geometry:test
       /misc:logging
       /parser/obj_parser:test
)

include(CTest)
include(Catch)
bottle_expand(TEST_BINARY :catch2_main)
catch_discover_tests(${TEST_BINARY})
