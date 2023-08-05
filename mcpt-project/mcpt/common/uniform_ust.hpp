#pragma once

#include <cmath>
#include <array>

#include <Eigen/Eigen>

#include "mcpt/common/random.hpp"

namespace mcpt {

template <typename T>
struct Direction {
  Eigen::Matrix<T, 3, 1> dir;
  double pdf;
};

// uniform sampling on unit spherical triangle
template <typename T>
class UniformUST {
public:
  using Scalar = T;

  template <typename U>
  Direction<T> Random(const U& a, const U& b, const U& c, T cos_alpha, T cos_beta, T cos_gamma) {
    T sin_alpha = std::sqrt(1.0F - cos_alpha * cos_alpha);
    T alpha = std::acos(cos_alpha);
    T beta = std::acos(cos_beta);
    T gamma = std::acos(cos_gamma);

    // use one random variable to select the new area
    T area = alpha + beta + gamma - M_PI;
    T area_sample = m_u1.Random() * area;

    // compute the pair (u, v) that determines beta'
    T s = std::sin(area_sample - alpha);
    T t = std::cos(area_sample - alpha);
    T u = t - cos_alpha;
    T v = s + sin_alpha * a.dot(b);

    // let q be the cosine of the new edge length b'
    T q = ((v * t - u * s) * cos_alpha - v) / ((v * s + u * t) * sin_alpha);

    // compute the third vertex of the sub-triangle
    Eigen::Matrix<T, 3, 1> cc = q * a + std::sqrt(1.0 - q * q) * (c - c.dot(a) * a).normalized();

    // use the other random variable to select cos(theta)
    T z = 1.0 - m_u2.Random() * (1.0 - cc.dot(b));

    // construct the corresponding point on the sphere
    Eigen::Matrix<T, 3, 1> p = z * b + std::sqrt(1.0 - z * z) * (cc - cc.dot(b) * b).normalized();
    return {p.normalized(), 1.0 / area};
  }

private:
  Uniform<T> m_u1;
  Uniform<T> m_u2;
};

}  // namespace mcpt
