set(EIGEN_BUILD_DOC OFF CACHE INTERNAL "")
set(BUILD_TESTING OFF CACHE INTERNAL "")
FetchContent_Declare(com.gitlab.libeigen.eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
    GIT_SHALLOW    TRUE)
FetchContent_MakeAvailable(com.gitlab.libeigen.eigen)

bottle_library(
  NAME eigen
  DEPS Eigen3::Eigen
)
