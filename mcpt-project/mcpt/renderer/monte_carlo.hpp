#pragma once

#include <vector>

#include <Eigen/Eigen>

#include "mcpt/common/geometry/types.hpp"
#include "mcpt/common/object/object.hpp"

#include "mcpt/renderer/path_tracer.hpp"

namespace mcpt {

// Monte Carlo integrator class
// - use BRDF(Bidirectional reflection distribution function) of Blinn-Phong model
// - and BTDF(Bidirectional transmittance distribution function) of Lambert model
class MonteCarlo {
public:
  struct options {
    double trunc_pdf = 1.0e-10;
    // camera options
    Eigen::Vector4f intrin{Eigen::Vector4f::Zero()};
    Eigen::Matrix3f R{Eigen::Matrix3f::Zero()};
    Eigen::Vector3f t{Eigen::Vector3f::Zero()};
  };

  MonteCarlo(options options, const Object& object) : m_options(options), m_path_tracer(object) {
    float fx = m_options.intrin.x();
    float fy = m_options.intrin.y();
    float cx = m_options.intrin.z();
    float cy = m_options.intrin.w();
    m_cam_K_inv << 1.0F / fx, 0.0F, -cx / fx, 0.0F, 1.0F / fy, -cy / fy, 0.0F, 0.0F, 1.0F;
  }

  Eigen::Vector3f Run(unsigned int u, unsigned int v);

private:
  struct Paths {
    double rpath_pdf;
    std::vector<PathTracer::ReversePath> rpaths;
  };

  // backtrace from the eye until:
  // - escaping from the scene (no more intersection)
  // - falling-off (the PDF is too small)
  Paths Backtrace(const Eigen::Vector3f& xy1);
  // propagate the light from the light source
  Eigen::Vector3f Propagate(const Eigen::Vector3f& eye, const Paths& paths) const;

  options m_options;
  Eigen::Matrix3f m_cam_K_inv;
  PathTracer m_path_tracer;
};

}  // namespace mcpt
