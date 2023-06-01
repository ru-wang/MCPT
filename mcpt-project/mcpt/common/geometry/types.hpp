#pragma once

#include <iterator>
#include <vector>

#include <Eigen/Eigen>

#include "mcpt/common/assert.hpp"

namespace mcpt {

template <typename T>
struct Line {
  using Scalar = T;
  Eigen::Matrix<T, 3, 1> point_a;
  Eigen::Matrix<T, 3, 1> point_b;

  template <typename A, typename B>
  Line(const Eigen::MatrixBase<A>& a, const Eigen::MatrixBase<B>& b) : point_a(a), point_b(b) {}
};

template <typename T>
struct Ray : Line<T> {
  using Scalar = T;
  Eigen::Matrix<T, 3, 1> direction;

  template <typename P, typename D>
  Ray(const Eigen::MatrixBase<P>& start_point, const Eigen::MatrixBase<D>& d)
      : Line<T>(start_point, start_point + d), direction(d.normalized()) {}
};

template <typename T>
struct Plane {
  using Scalar = T;
  Eigen::Matrix<T, 4, 1> coeffs;

  template <typename U>
  Plane(const Eigen::MatrixBase<U>& coeffs) : coeffs(coeffs) {}

  template <typename T0, typename T1, typename T2>
  Plane(const Eigen::MatrixBase<T0>& a,
        const Eigen::MatrixBase<T1>& b,
        const Eigen::MatrixBase<T2>& c) {
    Eigen::Matrix<T, 3, 1> d1 = b - a;
    Eigen::Matrix<T, 3, 1> d2 = c - a;
    coeffs.template head<3>() = d1.cross(d2).normalized();
    coeffs.w() = -a.dot(coeffs.template head<3>());
  }
};

// We take the first 3 points to compute the plane coefficients. The following points should be
// consistency with the first 3 points, otherwise the behaviour is undefined.
template <typename T>
struct ConvexPolygon : Plane<T> {
  using Scalar = T;
  std::vector<Eigen::Matrix<T, 3, 1>> vertices;

  template <typename T0, typename T1, typename T2, typename... Ts>
  ConvexPolygon(const Eigen::MatrixBase<T0>& a,
                const Eigen::MatrixBase<T1>& b,
                const Eigen::MatrixBase<T2>& c,
                const Eigen::MatrixBase<Ts>&... vs)
      : Plane<T>(a, b, c), vertices{a, b, c, vs...} {}

  template <typename InputIt>
  ConvexPolygon(InputIt first, InputIt last)
      : Plane<T>(*first, *(first + 1), *(first + 2)), vertices(first, last) {
    ASSERT(std::distance(first, last) >= 3);
  }
};

}  // namespace mcpt
