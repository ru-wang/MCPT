bottle_package()

bottle_library(
  NAME brdf
  SRCS brdf.cpp
  HDRS brdf.hpp
  DEPS @eigen
       //mcpt/common/object
)

bottle_library(
  NAME monte_carlo
  SRCS monte_carlo.cpp
  HDRS monte_carlo.hpp
  DEPS @eigen
       @spdlog
       //mcpt/common/geometry
       //mcpt/common/object
       :path_tracer
)

bottle_library(
  NAME path_tracer
  SRCS path_tracer.cpp
  HDRS path_tracer.hpp
  DEPS @eigen
       //mcpt/common/geometry
       //mcpt/common/object
       //mcpt/common:assert
       //mcpt/common:random
       :ray_caster
)

bottle_library(
  NAME ray_caster
  SRCS ray_caster.cpp
  HDRS ray_caster.hpp
  DEPS @eigen
       //mcpt/common/geometry
       //mcpt/common/object
)
