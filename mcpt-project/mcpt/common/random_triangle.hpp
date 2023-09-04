#pragma once

#include <cmath>

#include <Eigen/Eigen>

#include "mcpt/common/assert.hpp"
#include "mcpt/common/random.hpp"

namespace mcpt {

template <typename T>
class UniformTriangle {
public:
  using Scalar = T;

  template <typename U>
  Eigen::Matrix<T, 3, 1> Random(const U& a, const U& b, const U& c) {
    T sqrt_u1 = std::sqrt(m_u1.Random());
    T u2 = m_u2.Random();
    return (1.0 - sqrt_u1) * a + (sqrt_u1 * (1.0 - u2)) * b + (sqrt_u1 * u2) * c;
  }

private:
  Uniform<T> m_u1;
  Uniform<T> m_u2;
};

// uniform sampling on unit spherical triangle
template <typename T>
class UniformUnitSphericalTriangle {
public:
  using Scalar = T;

  template <typename U>
  Eigen::Matrix<T, 3, 1> Random(const U& a, const U& b, const U& c) {
    // assume inputs are normalized
    Eigen::Matrix<T, 3, 1> c_hat = slerp(a, c, m_u1.Random());
    T cos_arc_a_hat = b.dot(c_hat);
    T arc_a_hat = std::acos(cos_arc_a_hat);
    T t = std::acos(1.0 - m_u2.Random() * (1.0 - cos_arc_a_hat)) / arc_a_hat;
    return slerp(b, c_hat, t);
  }

private:
  template <typename U>
  static Eigen::Matrix<T, 3, 1> slerp(const U& from, const U& to, T t) {
    // assume inputs are normalized
    // [0, pi] is returned
    T cos = from.dot(to);
    ASSERT(cos >= -1.0 && cos <= 1.0);
    T theta = std::acos(cos);

    T sin_1_t_theta = std::sin((1.0 - t) * theta);
    T sin_t_theta = std::sin(t * theta);
    T sin_theta = std::sin(theta);

    ASSERT(std::isfinite(sin_theta) && sin_theta != 0.0, "t={} theta={}", t, theta);

    // output is normalized
    return ((sin_1_t_theta * from + sin_t_theta * to) / sin_theta).normalized();
  }

  Uniform<T> m_u1;
  Uniform<T> m_u2;
};

}  // namespace mcpt
