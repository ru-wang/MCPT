#include "mcpt/renderer/monte_carlo.hpp"

#include <cmath>

#include "mcpt/common/assert.hpp"
#include "mcpt/common/object/material.hpp"

namespace mcpt {

MonteCarlo::Result MonteCarlo::Run(unsigned int u, unsigned int v) {
  Eigen::Vector2f uv(u + m_uni_subpixel.Random(), v + m_uni_subpixel.Random());
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
    auto rpath = m_path_tracer.Run(ray);
    // stop if no intersection
    if (!rpath.has_value())
      return rpaths;

    // only sample direct lighting for diffusion material
    if (Material::Type(rpath.value().material) == Material::DIFF) {
      auto lpath = m_light_sampler.Run(rpath.value().point, rpath.value().normal);
      rpaths.push_back({rpath.value(), lpath});
    } else {
      rpaths.push_back({rpath.value()});
    }

    // stop if hit a light source
    if (Material::Type(rpath.value().material) == Material::EM)
      return rpaths;

    // stop if russian roulette fail
    if (m_russian_roulette.Random() >= m_options.rr_cont_prob)
      return rpaths;

    // generate next ray
    ray = Ray<float>(rpath.value().point, rpath.value().exit_dir);
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
  Eigen::Vector3f radiance = Eigen::Vector3f::Zero();

  // back propagate from the light source
  for (auto rit = rpaths.crbegin(); rit != rpaths.crend(); ++rit) {
    const auto& [rpath, lpath] = *rit;

    Eigen::Vector3f wo;
    if (rit + 1 == rpaths.crend())
      wo = (eye - rpath.point).normalized();
    else
      wo = -(rit + 1)->rpath.exit_dir;

    switch (Material::Type(rpath.material)) {
      case Material::EM: {
        // ray hit the light source directly
        DASSERT(rit == rpaths.crbegin());
        radiance = shade_light(wo, rpath);
      } break;

      case Material::TR:
      case Material::SPEC: {
        // only compute contribution from other reflectors & refractors
        radiance = shade_indirect(radiance, wo, rpath) / m_options.rr_cont_prob;
      } break;

      case Material::DIFF: {
        // contribution from the light sources
        Eigen::Vector3f r_direct = Eigen::Vector3f::Zero();
        if (lpath.has_value())
          r_direct = shade_direct(wo, rpath, lpath.value());

        // contribution from other reflectors & refractors
        Eigen::Vector3f r_indirect = Eigen::Vector3f::Zero();
        if (rit != rpaths.crbegin() && Material::Type((rit - 1)->rpath.material) != Material::EM)
          r_indirect = shade_indirect(radiance, wo, rpath) / m_options.rr_cont_prob;

        radiance = r_direct + r_indirect;
      } break;
    }

    DASSERT((radiance.array() >= 0.0F).all(), "wrong path radiance: {}", radiance.format(FMT));
  }

  return radiance;
}

Eigen::Vector3f MonteCarlo::shade_light(const Eigen::Vector3f& wo,
                                        const PathTracer::ReversePath& rpath) const {
  if (rpath.normal.dot(wo) > 0.0F)
    return Material::AsEmission(rpath.material);
  else
    return Eigen::Vector3f::Zero();
}

Eigen::Vector3f MonteCarlo::shade_direct(const Eigen::Vector3f& wo,
                                         const PathTracer::ReversePath& rpath,
                                         const LightSampler::PathToLight& lpath) const {
  Eigen::Vector3f fr = m_bxdf->Shade(rpath.material, rpath.normal, lpath.hit_dir, wo);
  float cos_wi = std::max(0.0F, rpath.normal.dot(lpath.hit_dir));
  return fr.cwiseProduct(Material::AsEmission(lpath.material)) * (cos_wi / lpath.hit_pdf);
}

Eigen::Vector3f MonteCarlo::shade_indirect(const Eigen::Vector3f& radiance,
                                           const Eigen::Vector3f& wo,
                                           const PathTracer::ReversePath& rpath) const {
  Eigen::Vector3f fr = m_bxdf->Shade(rpath.material, rpath.normal, rpath.exit_dir, wo);
  float cos_wi = std::max(0.0F, rpath.normal.dot(rpath.exit_dir));
  return fr.cwiseProduct(radiance) * (cos_wi / rpath.exit_pdf);
}

}  // namespace mcpt
