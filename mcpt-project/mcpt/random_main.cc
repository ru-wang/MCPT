#include <cmath>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Eigen>

#include "mcpt/common/assert.hpp"
#include "mcpt/common/geometry/types.hpp"
#include "mcpt/common/random.hpp"
#include "mcpt/common/random_triangle.hpp"
#include "mcpt/misc/logging.hpp"
#include "mcpt/misc/visualizing.hpp"

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

template <typename RndGen>
std::vector<Eigen::Vector3f> SampleTriangle(const Eigen::Vector3f& a,
                                            const Eigen::Vector3f& b,
                                            const Eigen::Vector3f& c,
                                            RndGen& gen,
                                            size_t num) {
  std::vector<Eigen::Vector3f> points;
  while (num--)
    points.emplace_back(gen.Random(a, b, c));
  return points;
}

int main(int argc, char* argv[]) {
  misc::InitLogger("random_main", ".", false);
  auto viz = misc::InitVisualizer(true);

  ASSERT(argc == 3);
  float arg_cos_pow = std::stof(argv[1]);
  size_t arg_samples = std::stoul(argv[2]);

  srand(std::random_device{}());
  Eigen::Vector3f a = (Eigen::Vector2f::Random() * 10.0F).homogeneous();
  Eigen::Vector3f b = (Eigen::Vector2f::Random() * 10.0F).homogeneous();
  Eigen::Vector3f c = (Eigen::Vector2f::Random() * 10.0F).homogeneous();

  Eigen::Vector3f center = (a + b + c) / 3.0F;
  Eigen::Vector3f normal = (b - a).cross(c - b).normalized();
  normal /= normal.z();

  CosPowHemisphere<float> cos_pow_hemi;
  auto lines = SampleLines(center, normal, cos_pow_hemi, arg_cos_pow, arg_samples);
  lines.emplace_back(a, b);
  lines.emplace_back(b, c);
  lines.emplace_back(c, a);

  UniformTriangle<float> uni_tri;
  auto points = SampleTriangle(a, b, c, uni_tri, arg_samples);

  UniformUnitSphericalTriangle<float> uni_sph_tri;
  auto directions =
      SampleTriangle(a.normalized(), b.normalized(), c.normalized(), uni_sph_tri, arg_samples);
  lines.emplace_back(Eigen::Vector3f::Zero(), a);
  lines.emplace_back(Eigen::Vector3f::Zero(), b);
  lines.emplace_back(Eigen::Vector3f::Zero(), c);

  viz.shape_layer->AddLines(lines);
  viz.shape_layer->AddPoints(points);
  viz.shape_layer->AddPoints(directions);
  viz.Run();

  return 0;
}
