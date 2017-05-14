#ifndef MCPT_RANDOM_H_
#define MCPT_RANDOM_H_

#include <cmath>
#include <random>

/*
 * Consine weighted distribution for float.
 *
 * Generates a consine weighted distributed variable from [0, π/2].
 * Also provides a function to compute the PDF for a given value.
 */
class Cosine {
 public:
  Cosine(int seed = 0) : generator_(seed), uniform_(0, 1) {}

  /*
   * When no parameter is given, generates the next random value.
   *
   * Assuem X~p(X) is a uniform distributed variable,
   * where p(x)=1, then let Y=arcsin(X) is subjected to
   * the new distribution Y~P(Y), where p(y) = cos(y).
   */
  float operator()() { return asin(uniform_(generator_)); }

  /*
   * Compute the PDF for a given value.
   *
   * The input should be in the range of [0, π/2].
   * Otherwise the ouput will be zero.
   */
  float operator()(float y) { return cos(y); }

 private:
  std::mt19937 generator_;
  std::uniform_real_distribution<float> uniform_;
};

#endif  /* MCPT_RANDOM_H_ */
