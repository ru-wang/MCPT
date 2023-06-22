bottle_package()

bottle_library(
  NAME aabb
  HDRS aabb.hpp
  DEPS @eigen
)

bottle_library(
  NAME bvh_tree
  HDRS bvh_tree.hpp
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
       :intersect_test
  XCLD
)
