#ifndef MCPT_RANDOM_H_
#define MCPT_RANDOM_H_

#include <cmath>
#include <ctime>
#include <random>
#include <tuple>

/*
 * Generic random distribution class for float.
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
  virtual float operator()() = 0;


  /*
   * Computes the PDF for a given value.
   *
   * All derived classes should override this function.
   */
  virtual float operator()(float y) const = 0;

 protected:
  std::mt19937 generator_;
  std::uniform_real_distribution<float> uniform_;
};

/*
 * Uniform distribution for float.
 */
class Uniform : public Distribution {
 public:
   Uniform(float range = 1, int seed = 0) : Distribution(seed), range_(range) {}

   virtual ~Uniform() override {}

  /*
   * When no parameter is given, generates the next random value.
   */
  virtual float operator()() override { return uniform_(generator_) * range_; }

  /*
   * Computes the PDF for a given value.
   */
  virtual float operator()(float /* y */) const override { return 1 / range_; }

 private:
  const float range_;
};

/*
 * Consine distribution for float.
 * Y~p(Y) where p(y)=cos(y).
 *
 * Generates a consine distributed variable from [0, π/2].
 * Also provides a function to compute the PDF for a given value.
 */
class Cosine : public Distribution {
 public:
  Cosine(int seed = 0) : Distribution(seed) {}

  virtual ~Cosine() override {}

  /*
   * When no parameter is given, generates the next random value.
   *
   * Assuem X~p(X) is a uniform distributed variable,
   * where p(x)=1, then Y=arcsin(X) is subjected to the
   * new distribution Y~p(Y), where p(y) = cos(y).
   */
  virtual float operator()() override { return asin(uniform_(generator_)); }

  /*
   * Computes the PDF for a given value.
   *
   * The input should be in the range of [0, π/2].
   * Otherwise the ouput will be zero.
   */
  virtual float operator()(float y) const override { return cos(y); }
};

/*
 * The uniform weigted distribution for float.
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
  std::tuple<float, float, float> operator()() {
    float u1 = u_();
    float u2 = u_();
    float phi = 2 * M_PI * u1;
    float theta = M_PI / 2 * u2;
    float p1 = 1 / (2 * M_PI);
    float p2 = 2 / M_PI;
    float p = p1 * p2;
    return std::make_tuple(phi, theta, p);
  }

 private:
  Uniform u_;  /* uniform generator for φ and θ */
};

/*
 * The consine weigted distribution for float.
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
  std::tuple<float, float, float> operator()() {
    float u1 = u_();
    float u2 = u_();
    float phi = 2 * M_PI * u1;
    float theta = asin(sqrt(u2));
    float p1 = 1 / (2 * M_PI);
    float p2 = sin(2 * theta);
    float p = p1 * p2;
    return std::make_tuple(phi, theta, p);
  }

 private:
  Uniform u_;  /* uniform generator for φ and θ */
};

/*
 * The cosᵃ weigted distribution for float.
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
  std::tuple<float, float, float> operator()(float a) {
    float u1 = u_();
    float u2 = u_();
    float phi = 2 * M_PI * u1;
    float theta = acos(pow(u2, 1 / (a + 1)));
    float p1 = 1 / (2 * M_PI);
    float p2 = (a + 1) * sin(theta) * pow(cos(theta), a);
    float p = p1 * p2;
    return std::make_tuple(phi, theta, p);
  }

 private:
  Uniform u_;  /* uniform generator for φ and θ */
};

#endif  /* MCPT_RANDOM_H_ */
