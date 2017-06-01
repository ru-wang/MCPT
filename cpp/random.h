#ifndef MCPT_RANDOM_H_
#define MCPT_RANDOM_H_

#include <cmath>
#include <ctime>
#include <random>
#include <tuple>

/*
 * Generic random distribution class for double.
 *
 * Generates a random variable subjected to a specific distribution.
 * Also provides a function to compute the PDF for a given value.
 */
class Distribution {
 public:
  Distribution(int seed) : generator_(seed), uniform_(0, 1) {}

  virtual ~Distribution() {}

  /*
   * When no parameter is given, generates the next random value.
   *
   * All derived classes should override this function.
   */
  virtual double operator()() = 0;


  /*
   * Computes the PDF for a given value.
   *
   * All derived classes should override this function.
   */
  virtual double operator()(double y) const = 0;

 protected:
  std::mt19937 generator_;
  std::uniform_real_distribution<double> uniform_;
};

/*
 * Uniform distribution for double.
 */
class Uniform : public Distribution {
 public:
   Uniform(double range = 1, int seed = 0) : Distribution(seed), range_(range) {}

   virtual ~Uniform() override {}

  /*
   * When no parameter is given, generates the next random value.
   */
  virtual double operator()() override { return uniform_(generator_) * range_; }

  /*
   * Computes the PDF for a given value.
   */
  virtual double operator()(double /* y */) const override { return 1 / range_; }

 private:
  const double range_;
};

/*
 * The uniform weigted distribution for double.
 *
 * Generates a (φ,θ) in a hemisphere surface region.
 * p(φ,θ)=p(φ)p(θ) where
 *   p(φ)=1/(2π), p(θ)=2/π.
 */
class UniformWeightedHemisphere {
 public:
  UniformWeightedHemisphere(int seed = std::time(nullptr)) : u_(1, seed) {}

  /*
   * Returns a (φ,θ,p) tuple where p=p(φ,θ)=p(φ)p(θ).
   */
  std::tuple<double, double, double> operator()() {
    double u1 = u_();
    double u2 = u_();
    double phi = kDoublePi * u1;
    double theta = kHalfPi * u2;
    double p1 = kDoublePiInv;
    double p2 = kHalfPiInv;
    double p = p1 * p2;
    return std::make_tuple(phi, theta, p);
  }

 private:
  Uniform u_;  /* uniform generator for φ and θ */

  static constexpr double kDoublePi = 2 * M_PI;
  static constexpr double kDoublePiInv = 1 / kDoublePi;
  static constexpr double kHalfPi = M_PI / 2;
  static constexpr double kHalfPiInv = 1 / kHalfPi;
};

/*
 * The consine weigted distribution for double.
 *
 * Generates a (φ,θ) in a hemisphere surface region.
 * p(φ,θ)=p(φ)p(θ) where
 *   p(φ)=1/(2π), p(θ)=2cosθsinθ.
 */
class CosineWeightedHemisphere {
 public:
  CosineWeightedHemisphere(int seed = std::time(nullptr)) : u_(1, seed) {}

  /*
   * Returns a (φ,θ,p) tuple where p=p(φ,θ)=p(φ)p(θ).
   */
  std::tuple<double, double, double> operator()() {
    double u1 = u_();
    double u2 = u_();
    double phi = kDoublePi * u1;
    double theta = std::asin(std::sqrt(u2));
    double p1 = kDoublePiInv;
    double p2 = 2 * std::sin(theta) * std::cos(theta);
    double p = p1 * p2;
    return std::make_tuple(phi, theta, p);
  }

 private:
  Uniform u_;  /* uniform generator for φ and θ */

  static constexpr double kDoublePi = 2 * M_PI;
  static constexpr double kDoublePiInv = 1 / kDoublePi;
};

/*
 * The cosᵃ weigted distribution for double.
 *
 * Generates a (φ,θ) in a hemisphere surface region.
 * p(φ,θ)=p(φ)p(θ) where
 *   p(φ)=1/(2π), p(θ)=(a+1)sinθcosᵃθ.
 *
 */
class CosinePowerWeightedHemisphere {
 public:
  CosinePowerWeightedHemisphere(int seed = std::time(nullptr)) : u_(1, seed) {}

  /*
   * Returns a (φ,θ,p) tuple where p=p(φ,θ)=p(φ)p(θ).
   */
  std::tuple<double, double, double> operator()(double a) {
    double u1 = u_();
    double u2 = u_();
    double phi = kDoublePi * u1;
    double cos_theta = pow(u2, 1 / (a + 1));
    double theta = acos(cos_theta);
    double p1 = kDoublePiInv;
    double p2 = (a + 1) * sin(theta) * pow(cos_theta, a);
    double p = p1 * p2;
    return std::make_tuple(phi, theta, p);
  }

 private:
  Uniform u_;  /* uniform generator for φ and θ */

  static constexpr double kDoublePi = 2 * M_PI;
  static constexpr double kDoublePiInv = 1 / kDoublePi;
};

#endif  /* MCPT_RANDOM_H_ */
