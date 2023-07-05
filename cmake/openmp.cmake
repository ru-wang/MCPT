find_package(OpenMP)

bottle_library(
  NAME openmp
  DEPS OpenMP::OpenMP_CXX
)
