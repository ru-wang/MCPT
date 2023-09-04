bottle_package()

bottle_library(
  NAME argparsing
  SRCS argparsing.cpp
  HDRS argparsing.hpp
  DEPS @argparse
)

bottle_library(
  NAME dispatcher
  SRCS dispatcher.cpp
  HDRS dispatcher.hpp
  DEPS @eigen
       @spdlog
       //mcpt/common
       //mcpt/renderer
)

bottle_library(
  NAME logging
  SRCS logging.cpp
  HDRS logging.hpp
  DEPS @spdlog
)

bottle_library(
  NAME visualizing
  SRCS visualizing.cpp
  HDRS visualizing.hpp
  DEPS //mcpt/common/viz
       @cheers
)
