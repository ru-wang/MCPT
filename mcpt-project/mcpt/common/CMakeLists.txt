bottle_package()

bottle_subdir(NAME fileserver)
bottle_subdir(NAME geometry)
bottle_subdir(NAME object)
bottle_subdir(NAME viz)

bottle_library(
  NAME assert
  HDRS assert.hpp
  DEPS @spdlog
  OPTS -Wno-gnu-zero-variadic-macro-arguments
)

bottle_library(
  NAME misc
  SRCS misc.cpp
  HDRS misc.hpp
  DEPS @spdlog
       :assert
)

bottle_library(
  NAME random
  HDRS random.hpp
)

bottle_library(
  NAME uniform_ust
  HDRS uniform_ust.hpp
  DEPS :random
)
