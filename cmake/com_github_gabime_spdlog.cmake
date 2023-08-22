set(SPDLOG_BUILD_SHARED ON CACHE INTERNAL "")
FetchContent_Declare(com.github.gabime.spdlog
  GIT_REPOSITORY https://github.com/gabime/spdlog.git
  GIT_TAG        v1.11.0
  GIT_SHALLOW    TRUE)
FetchContent_MakeAvailable(com.github.gabime.spdlog)

bottle_library(
  NAME spdlog
  DEPS spdlog::spdlog
)
