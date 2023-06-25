bottle_package()

bottle_library(
  NAME aabb
  HDRS aabb.hpp
  DEPS @eigen
       //mcpt/common:assert
)

bottle_library(
  NAME bvh_tree
  HDRS bvh_node.hpp
       bvh_tree.hpp
  DEPS @eigen
       //mcpt/common:assert
       :aabb
)

bottle_library(
  NAME intersect
  HDRS intersect.hpp
  DEPS @eigen
       @spdlog
       :aabb
       :types
)

bottle_library(
  NAME types
  HDRS types.hpp
  DEPS @eigen
       //mcpt/common:assert
)

bottle_library(
  NAME aabb_test
  SRCS aabb_test.cpp
  DEPS @catch2
       @eigen
       :aabb
  XCLD
)

bottle_library(
  NAME bvh_tree_test
  SRCS bvh_tree_test.cpp
  DEPS @catch2
       //mcpt/common/object
       //mcpt/parser/obj_parser:parser
       //mcpt/parser/obj_parser:test_helper
       :bvh_tree
  XCLD
)

bottle_library(
  NAME intersect_test
  SRCS intersect_test.cpp
  DEPS @catch2
       @eigen
       :aabb
       :intersect
       :types
  XCLD
)

bottle_library(
  NAME test
  DEPS :aabb_test
       :bvh_tree_test
       :intersect_test
  XCLD
)
