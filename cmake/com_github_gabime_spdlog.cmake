FetchContent_Declare(com.github.gabime.spdlog
  GIT_REPOSITORY https://github.com/gabime/spdlog.git
  GIT_TAG        v1.x
  GIT_SHALLOW    TRUE)
FetchContent_MakeAvailable(com.github.gabime.spdlog)

bottle_library(
  NAME spdlog
  OPTS -Wno-dangling-reference
  DEPS spdlog::spdlog
)
