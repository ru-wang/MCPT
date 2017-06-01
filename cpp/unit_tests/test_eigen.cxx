#include "eigen.h"

#include <iomanip>
#include <iostream>
#include <string>
#include <sstream>
#include <typeinfo>
#include <vector>

using namespace std;

namespace {

template <typename T, size_t D>
string ToString(const Vector<T, D>& vec) {
  stringstream ss;
  ss << "Vector" << D << typeid(T).name() << " [ ";
  for (size_t i = 0; i < D; ++i)
    ss << setprecision(4) << setw(4) << vec[i] << " ";
  ss << "]";
  return ss.str();
}

template <typename T, size_t D>
string ToString(const Matrix<T, D>& mat) {
  stringstream ss;
  ss << "Matrix" << D << typeid(T).name() << "\n";
  for (size_t i = 0; i < D; ++i) {
    ss << "| ";
    for (size_t j = 0; j < D; ++j)
      ss << setprecision(4) << setw(4) << mat(i, j) << " ";
    ss << "|\n";
  }
  ss << "\n";
  return ss.str();
}

}

int main() {
  cerr << ToString(Vector<double, 4>(1, 2))       << "\n"
       << ToString(Vector<double, 4>(1, 2, 3))    << "\n"
       << ToString(Vector<double, 4>(1, 2, 3, 4)) << "\n"
       << ToString(Vector<double, 4>{1, 2})       << "\n"
       << ToString(Vector<double, 4>{1, 2, 3})    << "\n"
       << ToString(Vector<double, 4>{1, 2, 3, 4}) << "\n"
       << "\n"
       << ToString(Vector<double, 4>::Zero())     << "\n"
       << ToString(Vector<double, 4>::Zero() + Vector<double, 4>::All(8)) << "\n"
       << ToString(Vector<double, 4>::Ones())     << "\n"
       << "\n"
       << ToString(Vector<double, 4>(1, 2, 3, 4) * 10) << "\n"
       << ToString(Vector<double, 4>(1, 2, 3, 4) / 10) << "\n"
       << Vector<double, 4>::Ones() * Vector<double, 4>(1, 2, 3, 4) << "\n"
       << ToString(Cross(Vector<double, 3>(0, 1, 0),
                         Vector<double, 3>(1, 0, 0))) << "\n"
       << ToString(Vector<double, 3>(1, 0, 0).Cross(
                   Vector<double, 3>(0, 1, 0))) << "\n"
       << "\n"
       << Vector<double, 3>(1, 0, 1).L0() << "\n"
       << Vector<double, 3>(1, -1, 1).L1() << "\n"
       << Vector<double, 3>(1, 1, 1).L2() << "\n"
       << ToString(Vector<double, 3>(1, 2, 2).Normalize()) << "\n"
       << ToString(Vector<double, 4>(1, 2, 3, 4).Normalize()) << "\n"
       << "\n"
       << Vector<double, 3>(1, 0, 0).Cos(Vector<double, 3>(1, 1, 0)) << "\n"
       << Cos(Vector<double, 3>(1, 0, 0), Vector<double, 3>(0, 1, 0)) << "\n"
       << "\n"
       << ToString(Matrix<double, 4>::Identity()) << "\n"
       << ToString(Matrix<double, 4>::Diagonal(Vector<double, 4>(1, 2, 3, 4))) << "\n"
       << ToString(Matrix<double, 4>::Diagonal(Vector<double, 4>(1, 2, 3, 4)) *
                   Vector<double, 4>(1, 2, 3, -4)) << "\n"
       ;
  return 0;
}
