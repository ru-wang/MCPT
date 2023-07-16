#pragma once

#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Eigen>

#include "mcpt/common/geometry/bvh_tree.hpp"
#include "mcpt/common/geometry/types.hpp"
#include "mcpt/common/object/object.hpp"
#include "mcpt/common/random.hpp"

#include "mcpt/renderer/brdf.hpp"
#include "mcpt/renderer/path_tracer.hpp"

namespace mcpt {

// Monte Carlo integrator class
// - use BRDF(Bidirectional reflection distribution function) of Blinn-Phong model
// - and BTDF(Bidirectional transmittance distribution function) of Lambert model
class MonteCarlo {
public:
  struct Result {
    std::vector<PathTracer::ReversePath> rpaths;
    Eigen::Vector3f radiance;
  };

  struct Options {
    size_t min_num_paths = 3;
    double rr_continue_prob = 0.5;
    // camera options
    Eigen::Vector4f intrin{Eigen::Vector4f::Zero()};
    Eigen::Matrix3f R{Eigen::Matrix3f::Zero()};
    Eigen::Vector3f t{Eigen::Vector3f::Zero()};
  };

  MonteCarlo(const Options& options, const Object& object, const BVHTree<float>& bvh_tree)
      : m_options(options), m_path_tracer(object, bvh_tree) {
    float fx = m_options.intrin.x();
    float fy = m_options.intrin.y();
    float cx = m_options.intrin.z();
    float cy = m_options.intrin.w();
    m_intrin_inv << 1.0F / fx, 0.0F, -cx / fx, 0.0F, 1.0F / fy, -cy / fy, 0.0F, 0.0F, 1.0F;
  }

  void InstallBRDF(std::unique_ptr<BRDF> brdf) { m_brdf = std::move(brdf); }

  Result Run(unsigned int u, unsigned int v);

private:
  using RPaths = std::vector<PathTracer::ReversePath>;

  // backtrace from the eye until:
  // - escaping from the scene (no more intersection)
  // - failing in Russian roulette
  RPaths Backtrace(const Eigen::Vector3f& xy1);
  // propagate the light from the light source
  Eigen::Vector3f Propagate(const Eigen::Vector3f& eye, const RPaths& rpaths) const;

  Options m_options;
  std::unique_ptr<BRDF> m_brdf;

  Eigen::Matrix3f m_intrin_inv;
  PathTracer m_path_tracer;
  Uniform<double> m_russian_roulette;
};

}  // namespace mcpt
