#pragma once

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include <Eigen/Eigen>

#include "mcpt/common/geometry/bvh_tree.hpp"
#include "mcpt/common/geometry/types.hpp"
#include "mcpt/common/object/object.hpp"
#include "mcpt/common/random.hpp"

#include "mcpt/renderer/bxdf.hpp"
#include "mcpt/renderer/light_sampler.hpp"
#include "mcpt/renderer/path_tracer.hpp"

namespace mcpt {

// Monte Carlo integrator class
class MonteCarlo {
public:
  struct RPath {
    PathTracer::ReversePath rpath;
    std::optional<LightSampler::PathToLight> lpath;
  };

  struct Result {
    std::vector<RPath> rpaths;
    Eigen::Vector3f radiance;
  };

  struct Options {
    double rr_cont_prob = 0.5;
    // camera options
    Eigen::Vector4f intrin{Eigen::Vector4f::Zero()};
    Eigen::Matrix3f R{Eigen::Matrix3f::Zero()};
    Eigen::Vector3f t{Eigen::Vector3f::Zero()};
  };

  MonteCarlo(const Options& options, const Object& object, const BVHTree<float>& bvh_tree)
      : m_options(options), m_path_tracer(object, bvh_tree), m_light_sampler(object, bvh_tree) {
    float fx = m_options.intrin.x();
    float fy = m_options.intrin.y();
    float cx = m_options.intrin.z();
    float cy = m_options.intrin.w();
    m_intrin_inv << 1.0F / fx, 0.0F, -cx / fx, 0.0F, 1.0F / fy, -cy / fy, 0.0F, 0.0F, 1.0F;
  }

  void SetBxDF(std::unique_ptr<BxDF> bxdf) { m_bxdf = std::move(bxdf); }

  Result Run(unsigned int u, unsigned int v);

private:
  using RPaths = std::vector<RPath>;

  // backtrace from the eye until:
  // - escaping from the scene (no more intersection)
  // - failing in Russian roulette
  RPaths Backtrace(const Eigen::Vector3f& xy1);
  // propagate the light from the light source
  Eigen::Vector3f Propagate(const Eigen::Vector3f& eye, const RPaths& rpaths) const;

  Eigen::Vector3f shade_light(const Eigen::Vector3f& wo,
                              const PathTracer::ReversePath& rpath) const;
  Eigen::Vector3f shade_direct(const Eigen::Vector3f& wo,
                               const PathTracer::ReversePath& rpath,
                               const LightSampler::PathToLight& lpath) const;
  Eigen::Vector3f shade_indirect(const Eigen::Vector3f& radiance,
                                 const Eigen::Vector3f& wo,
                                 const PathTracer::ReversePath& rpath) const;

private:
  Options m_options;
  std::unique_ptr<BxDF> m_bxdf;

  Eigen::Matrix3f m_intrin_inv;
  PathTracer m_path_tracer;
  LightSampler m_light_sampler;

  Uniform<float> m_uni_subpixel;
  Uniform<double> m_russian_roulette;
};

}  // namespace mcpt
