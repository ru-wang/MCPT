bottle_package()

bottle_subdir(NAME common)
bottle_subdir(NAME parser)
bottle_subdir(NAME renderer)

bottle_binary(
  NAME mcpt_main
  SRCS mcpt_main.cc
  DEPS /common/fileserver
       /common:assert
       /parser/obj_parser
       /parser:misc
       /renderer:monte_carlo
)

bottle_binary(
  NAME catch2_main
  SRCS catch2_main.cc
  DEPS @catch2
       @spdlog
       /common/geometry:test
       /parser/obj_parser:test
)

include(CTest)
include(Catch)
bottle_expand(TEST_BINARY :catch2_main)
catch_discover_tests(${TEST_BINARY})
