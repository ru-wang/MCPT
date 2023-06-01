#include "mcpt/renderer/monte_carlo.hpp"

#include <cmath>

#include <spdlog/spdlog.h>

namespace mcpt {

Eigen::Vector3f MonteCarlo::Run(unsigned int u, unsigned int v) {
  Eigen::Vector2f uv(u, v);
  Eigen::Vector3f xy1 = m_cam_K_inv * uv.homogeneous();
  spdlog::debug("compute contribution of uv({}, {})", u, v);
  return Propagate(m_options.t, Backtrace(xy1));
}

MonteCarlo::Paths MonteCarlo::Backtrace(const Eigen::Vector3f& xy1) {
  Eigen::Vector3f eye_dir = m_options.R * xy1;
  Ray<float> ray(m_options.t, eye_dir.normalized());

  for (Paths paths{1.0};;) {
    auto path = m_path_tracer.Run(ray);

    // no intersection
    if (!path.has_value())
      return paths;
    paths.rpaths.push_back(path.value());

    // fall off
    if (paths.rpath_pdf * path.value().exit_pdf < m_options.trunc_pdf)
      return paths;
    paths.rpath_pdf *= path.value().exit_pdf;

    ray = Ray<float>(path.value().point, path.value().exit_dir);
  }
}

Eigen::Vector3f MonteCarlo::Propagate(const Eigen::Vector3f& eye, const Paths& paths) const {
  if (paths.rpaths.empty())
    return Eigen::Vector3f::Zero();

  // light source
  Eigen::Vector3f I = paths.rpaths.front().target_material.get().Ka;

  // back propagate from the light source
  for (auto rit = paths.rpaths.crbegin() + 1; rit != paths.rpaths.crend(); ++rit) {
    const auto& rpath = *rit;
    const auto& src_mtl = rpath.target_material.get();

    const auto& N = rpath.normal;
    const auto& L = rpath.exit_dir;

    Eigen::Vector3f V;
    if (rit + 1 == paths.rpaths.crend())
      V = -(rit + 1)->exit_dir;
    else
      V = (m_options.t - rpath.point).normalized();

    Eigen::Vector3f H = (L + V).normalized();

    Eigen::Vector3f comp_diffusion = src_mtl.Tr * I.cwiseProduct(src_mtl.Kd) * N.dot(L);
    Eigen::Vector3f comp_specular = I.cwiseProduct(src_mtl.Ks) * std::pow(H.dot(N), src_mtl.Ns);

    I = src_mtl.Ka + comp_diffusion + comp_specular;
  }

  return I / paths.rpath_pdf;
}

}  // namespace mcpt
