bottle_package()

bottle_library(
  NAME material
  HDRS material.hpp
  DEPS @eigen
)

bottle_library(
  NAME mesh
  HDRS mesh.hpp
)

bottle_library(
  NAME object
  SRCS object.cpp
  HDRS object.hpp
  DEPS @eigen
       @spdlog
       //mcpt/common/geometry:bvh_tree
       //mcpt/common:assert
       :material
       :mesh
)
