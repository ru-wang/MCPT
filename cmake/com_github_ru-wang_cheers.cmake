set(CHEERS_BUILD_EXAMPLE OFF CACHE INTERNAL "")
FetchContent_Declare(com.github.ru-wang.cheers
  GIT_REPOSITORY git@github.com:ru-wang/cheers.git
  GIT_TAG        master
  GIT_SHALLOW    TRUE)
FetchContent_MakeAvailable(com.github.ru-wang.cheers)

bottle_library(
  NAME cheers
  DEPS cheers-project::cheers
)
