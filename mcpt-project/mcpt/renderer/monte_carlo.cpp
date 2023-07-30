#include "mcpt/renderer/monte_carlo.hpp"

#include <cmath>

#include "mcpt/common/assert.hpp"
#include "mcpt/common/object/material.hpp"

namespace mcpt {

MonteCarlo::Result MonteCarlo::Run(unsigned int u, unsigned int v) {
  Eigen::Vector2f uv(u, v);
  Eigen::Vector3f xy1 = m_intrin_inv * uv.homogeneous();

  Result result;
  result.rpaths = Backtrace(xy1);
  result.radiance = Propagate(m_options.t, result.rpaths);
  return result;
}

/**
 *                     _______________
 *                           /\             _______
 *                          /  \              /|
 *              \          /    \            / |
 *  eye..........\        /      \          /  |[4]
 *               |\   [1]/        \     [3]/   |
 *               |      /       [2]\      /    |
 *               |     /            \    /    \|_______________
 *            [0]|    /              \  /      \      [5]
 *               |   /           _____\/___     \
 *               |  /
 *               | /
 *           ____|/____
 *                        |rpaths| = 6
 */
MonteCarlo::RPaths MonteCarlo::Backtrace(const Eigen::Vector3f& xy1) {
  RPaths rpaths;
  Ray<float> ray(m_options.t, m_options.R * xy1);
  while (true) {
    auto path = m_path_tracer.Run(ray);
    // no intersection
    if (!path.has_value())
      return rpaths;
    rpaths.push_back(path.value());

    // test russian roulette and modify the PDF
    if (rpaths.size() >= m_options.min_num_paths) {
      bool rr_continue = m_russian_roulette.Random() < m_options.rr_continue_prob;
      rpaths.back().exit_pdf *=
          rr_continue ? m_options.rr_continue_prob : 1.0 - m_options.rr_continue_prob;
      // stop if russian roulette fail
      if (!rr_continue)
        return rpaths;
    }

    // generate next ray
    ray = Ray<float>(path.value().point, path.value().exit_dir);
  }
}

/**
 *           | N
 *  L \      |      / V
 *     \     |     /
 *      \    |    /
 *       \   |   /
 *        \  |  /
 *         \ | /
 *    ______\|/_______
 */
Eigen::Vector3f MonteCarlo::Propagate(const Eigen::Vector3f& eye, const RPaths& rpaths) const {
  static const Eigen::IOFormat FMT{Eigen::FullPrecision, Eigen::DontAlignCols, " ", " "};

  if (rpaths.empty())
    return Eigen::Vector3f::Zero();
  Eigen::Vector3f radiance = Material::AsEmission(rpaths.back().target_material);

  // back propagate from the light source
  for (auto rit = rpaths.crbegin() + 1; rit != rpaths.crend(); ++rit) {
    const auto& rpath = *rit;
    const auto& p_mtl = rpath.target_material.get();

    const auto& n = rpath.normal;
    const auto& wi = rpath.exit_dir;

    Eigen::Vector3f wo;
    if (rit + 1 == rpaths.crend())
      wo = (m_options.t - rpath.point).normalized();
    else
      wo = -(rit + 1)->exit_dir;

    float pdf_wi = rpath.exit_pdf;
    float cos_wi = std::abs(n.dot(wi));
    Eigen::Vector3f fr = m_bxdf->Shade(p_mtl, n, wi, wo);

    radiance = Material::AsEmission(p_mtl) + fr.cwiseProduct(radiance) * cos_wi / pdf_wi;
    DASSERT((radiance.array() >= 0.0F).all(), "wrong path radiance: {}", radiance.format(FMT));
  }

  return radiance;
}

}  // namespace mcpt
