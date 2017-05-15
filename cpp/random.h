#ifndef MCPT_RANDOM_H_
#define MCPT_RANDOM_H_

#include <cmath>
#include <random>

/*
 * Generic random distribution class for float.
 *
 * Generates a random variable subjected to a specific distribution.
 * Also provides a function to compute the PDF for a given value.
 */
class Distribution {
 public:
  /*
   * When no parameter is given, generates the next random value.
   *
   * All derived classes should override this function.
   */
  virtual float operator()() = 0;


  /*
   * Compute the PDF for a given value.
   *
   * All derived classes should override this function.
   */
  virtual float operator()(float y) = 0;
};

/*
 * Consine weighted distribution for float.
 *
 * Generates a consine weighted distributed variable from [0, π/2].
 * Also provides a function to compute the PDF for a given value.
 */
class Cosine : public Distribution {
 public:
  Cosine(int seed = 0) : generator_(seed), uniform_(0, 1) {}

  /*
   * When no parameter is given, generates the next random value.
   *
   * Assuem X~p(X) is a uniform distributed variable,
   * where p(x)=1, then let Y=arcsin(X) is subjected to
   * the new distribution Y~P(Y), where p(y) = cos(y).
   */
  virtual float operator()() override { return asin(uniform_(generator_)); }

  /*
   * Compute the PDF for a given value.
   *
   * The input should be in the range of [0, π/2].
   * Otherwise the ouput will be zero.
   */
  virtual float operator()(float y) override { return cos(y); }

 private:
  std::mt19937 generator_;
  std::uniform_real_distribution<float> uniform_;
};

#endif  /* MCPT_RANDOM_H_ */
