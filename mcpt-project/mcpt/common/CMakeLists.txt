bottle_package()

bottle_subdir(NAME fileserver)
bottle_subdir(NAME geometry)
bottle_subdir(NAME object)

bottle_library(
  NAME assert
  HDRS assert.hpp
  DEPS @spdlog
)

bottle_library(
  NAME random
  HDRS random.hpp
)
