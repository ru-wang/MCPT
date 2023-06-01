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
       :aabb
       :types
)

bottle_library(
  NAME types
  HDRS types.hpp
  DEPS @eigen
       //mcpt/common:assert
)
