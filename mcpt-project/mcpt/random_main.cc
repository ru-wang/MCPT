#include <cmath>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <cheers/window/window.hpp>

#include "mcpt/common/assert.hpp"
#include "mcpt/common/geometry/types.hpp"
#include "mcpt/common/random.hpp"
#include "mcpt/common/viz/shape_layer.hpp"

#include "mcpt/misc.hpp"

using namespace mcpt;

template <typename RndGen>
std::vector<Line<float>> SampleLines(const Eigen::Vector3f& center,
                                     const Eigen::Vector3f& normal,
                                     RndGen& gen,
                                     float alpha,
                                     size_t num) {
  Eigen::Index x;
  normal.cwiseAbs().minCoeff(&x);

  // find the plane coordinate system
  Eigen::Matrix3f axes;
  axes.col(2) = normal.normalized();
  axes.col(1) = axes.col(2).cross(Eigen::Vector3f::Unit(x)).normalized();
  axes.col(0) = axes.col(1).cross(axes.col(2)).normalized();

  std::vector<Line<float>> lines;
  while (num--) {
    auto sa = gen.Random(alpha);
    Eigen::Vector3f dir(std::sin(sa.depression) * std::cos(sa.azimuth),
                        std::sin(sa.depression) * std::sin(sa.azimuth),
                        std::cos(sa.depression));
    lines.emplace_back(center, axes * (dir * 10.0F) + center);
  }
  return lines;
}

int main(int argc, char* argv[]) {
  MakeLogger("random");

  ASSERT(argc == 3);
  float arg_cos_pow = std::stof(argv[1]);
  size_t arg_max_samples = std::stoul(argv[2]);

  srand(std::random_device{}());
  Eigen::Vector3f a = Eigen::Vector3f::Random() * 10.0F;
  Eigen::Vector3f b = Eigen::Vector3f::Random() * 10.0F;
  Eigen::Vector3f c = Eigen::Vector3f::Random() * 10.0F;

  Eigen::Vector3f normal = (b - a).cross(c - b).normalized();
  Eigen::Vector3f center = (a + b + c) / 3.0F;

  std::vector<Line<float>> lines;
  CosPowHemisphere<float> gen;
  lines = SampleLines(center, normal, gen, arg_cos_pow, arg_max_samples);

  lines.emplace_back(a, b);
  lines.emplace_back(b, c);
  lines.emplace_back(c, a);

  cheers::Window::Instance().InstallSharedLayer<ShapeLayer>()->AddLines(lines);
  cheers::Window::Instance().CreateContext();
  while (cheers::Window::Instance().WaitForWindowExiting())
    ;
  cheers::Window::Instance().DestroyContext();

  return 0;
}
