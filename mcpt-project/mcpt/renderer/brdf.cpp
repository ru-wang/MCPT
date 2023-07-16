#include "mcpt/renderer/brdf.hpp"

#include <cmath>

namespace mcpt {

Eigen::Vector3f BlinnPhongBRDF::Shade(const Material& p_mtl,
                                      const Eigen::Vector3f& n,
                                      const Eigen::Vector3f& wi, float wi_norm,
                                      const Eigen::Vector3f& wo, float wo_norm) const {
  if (p_mtl.Tr)
    return Eigen::Vector3f::Constant(p_mtl.Tr);

  Eigen::Vector3f halfway = (wi + wo).normalized();
  float cos_h = std::abs(n.dot(halfway));
  float pow_cos_h = std::pow(cos_h, p_mtl.Ns);

  Eigen::Vector3f diffusion = p_mtl.Kd / M_PI / 2.0F;
  Eigen::Vector3f specular = p_mtl.Ks * pow_cos_h;
  return diffusion + specular;
}

}  // namespace mcpt