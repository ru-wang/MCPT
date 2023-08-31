FetchContent_Declare(com.github.p-ranav.argparse
    GIT_REPOSITORY https://github.com/p-ranav/argparse.git
    GIT_TAG        v2.9
    GIT_SHALLOW    TRUE)
FetchContent_MakeAvailable(com.github.p-ranav.argparse)

bottle_library(
  NAME argparse
  DEPS argparse
)
