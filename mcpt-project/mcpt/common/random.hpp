#pragma once

#include <cmath>
#include <random>
#include <type_traits>

namespace mcpt {

template <typename T, typename Enabled = void>
class Uniform;

template <typename T>
class Uniform<T, std::enable_if_t<std::is_floating_point_v<T>>> {
public:
  using Scalar = T;

  T Random() {
#ifndef NDEBUG
    static thread_local std::mt19937 gen{0};
#else
    static thread_local std::mt19937 gen{std::random_device{}()};
#endif
    // [0,1)
    return std::uniform_real_distribution<T>{0, 1}(gen);
  }
};

template <typename T>
class Uniform<T, std::enable_if_t<std::is_integral_v<T>>> {
public:
  using Scalar = T;

  T Random(T min, T max) {
#ifndef NDEBUG
    static thread_local std::mt19937 gen{0};
#else
    static thread_local std::mt19937 gen{std::random_device{}()};
#endif
    // [min,max]
    return std::uniform_int_distribution<T>{min, max}(gen);
  }
};

template <typename T>
struct SolidAngle {
  T azimuth;     // [0, 2pi)
  T depression;  // [0, pi/2)
  double pdf;
};

// uniform weigted distribution on a hemisphere
template <typename T>
class UniformHemisphere {
public:
  using Scalar = T;

  static constexpr double TWO_PI = 2.0 * M_PI;

  SolidAngle<T> Random() {
    // sample uniformly [0, 2pi), PDF = 1/(2pi)
    T azimuth = TWO_PI * m_u1.Random();
    // sample [0, pi/2), PDF = sin(x), CDF = 1-cos(x), CDF^-1(x) = arccos(1-x)
    T depression = std::acos(1 - m_u2.Random());
    // PDF = 1/(2pi)
    return {azimuth, depression, 1.0 / TWO_PI};
  }

private:
  Uniform<T> m_u1;
  Uniform<T> m_u2;
};

// cosine weigted distribution on a hemisphere
template <typename T>
class CosHemisphere {
public:
  using Scalar = T;

  static constexpr double TWO_PI = 2.0 * M_PI;

  SolidAngle<T> Random() {
    // sample uniformly [0, 2pi), PDF=1/(2pi)
    T azimuth = TWO_PI * m_u1.Random();
    // sample [0, pi/2)
    // PDF = 2sin(x)cos(x) = sin(2x)
    // CDF = sin^2(x)
    // CDF^-1 = arccos(sqrt(u))
    T depression = std::asin(std::sqrt(m_u2.Random()));
    // PDF = cos(depression)/pi
    double pdf = std::cos(depression) / M_PI;
    return {azimuth, depression, pdf};
  }

private:
  Uniform<T> m_u1;
  Uniform<T> m_u2;
};

// cosine weigted distribution on a hemisphere
template <typename T>
class CosPowHemisphere {
public:
  using Scalar = T;

  static constexpr double TWO_PI = 2.0 * M_PI;

  SolidAngle<T> Random(T alpha) {
    T alpha_1 = alpha + 1.0;
    // sample uniformly [0,2pi), PDF=1/(2pi)
    T azimuth = TWO_PI * m_u1.Random();
    // sample [0, pi/2)
    // PDF = (a+1)cos^a(dpr)sin(dpr)/(2pi)
    // CDF = 1-cos^(a+1)(dpr)
    // CDF^-1 = arccos[(1-u)^(1/(a+1))]
    T depression = std::acos(std::pow(1.0 - m_u2.Random(), 1.0 / alpha_1));
    // PDF = (a+1)cos^a(depression)/(2pi)
    double pdf = alpha_1 * std::pow(std::cos(depression), alpha) / TWO_PI;
    return {azimuth, depression, pdf};
  }

private:
  Uniform<T> m_u1;
  Uniform<T> m_u2;
};

}  // namespace mcpt
