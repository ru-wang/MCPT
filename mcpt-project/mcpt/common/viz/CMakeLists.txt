bottle_package()

bottle_library(
  NAME object_layer
  SRCS object_layer.cpp
  HDRS object_layer.hpp
  DEPS @cheers
       @eigen
       //mcpt/common/object:object
)
