#pragma once

#include <cmath>
#include <random>

namespace mcpt {

template <typename T>
class Uniform {
public:
  using Scalar = T;

  Uniform() = default;
  Uniform(int seed) : m_gen(seed) {}

  T Random() { return m_uniform(m_gen); }

private:
  std::random_device m_rd;
  std::mt19937 m_gen{m_rd()};
  // [0,1)
  std::uniform_real_distribution<T> m_uniform{0, 1};
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

  UniformHemisphere() = default;
  UniformHemisphere(int seed) : m_u1(seed), m_u2(seed) {}
  UniformHemisphere(int seed_u1, int seed_u2) : m_u1(seed_u1), m_u2(seed_u2) {}

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

  CosHemisphere() = default;
  CosHemisphere(int seed) : m_u1(seed), m_u2(seed) {}
  CosHemisphere(int seed_u1, int seed_u2) : m_u1(seed_u1), m_u2(seed_u2) {}

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

  CosPowHemisphere() = default;
  CosPowHemisphere(int seed) : m_u1(seed), m_u2(seed) {}
  CosPowHemisphere(int seed_u1, int seed_u2) : m_u1(seed_u1), m_u2(seed_u2) {}

  SolidAngle<T> Random(float alpha) {
    float alpha_1 = alpha + 1.0F;
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
