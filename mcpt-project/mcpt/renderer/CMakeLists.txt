bottle_package()

bottle_library(
  NAME bxdf
  SRCS bxdf.cpp
  HDRS bxdf.hpp
  DEPS @eigen
       //mcpt/common/object
)

bottle_library(
  NAME light_sampler
  SRCS light_sampler.cpp
  HDRS light_sampler.hpp
  DEPS @eigen
       @spdlog
       //mcpt/common/geometry
       //mcpt/common/object
       //mcpt/common:assert
       //mcpt/common:random
)

bottle_library(
  NAME monte_carlo
  SRCS monte_carlo.cpp
  HDRS monte_carlo.hpp
  DEPS @eigen
       //mcpt/common/geometry
       //mcpt/common/object
       //mcpt/common:assert
       //mcpt/common:random
       :bxdf
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
