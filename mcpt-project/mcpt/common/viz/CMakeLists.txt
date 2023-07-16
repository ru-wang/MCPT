bottle_package()

bottle_library(
  NAME object_layer
  SRCS object_layer.cpp
  HDRS object_layer.hpp
  DEPS @cheers
       @eigen
       //mcpt/common/object:object
)

bottle_library(
  NAME path_layer
  SRCS path_layer.cpp
  HDRS path_layer.hpp
  DEPS @cheers
       @eigen
       //mcpt/common:assert
)

bottle_library(
  NAME shape_layer
  SRCS shape_layer.cpp
  HDRS shape_layer.hpp
  DEPS @cheers
       @eigen
       //mcpt/common/geometry:types
)
