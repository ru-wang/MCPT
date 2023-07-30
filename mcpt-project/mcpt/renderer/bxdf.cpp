#include "mcpt/renderer/bxdf.hpp"

#include <cmath>

namespace mcpt {

Eigen::Vector3f BlinnPhongBxDF::Shade(const Material& p_mtl,
                                      const Eigen::Vector3f& n,
                                      const Eigen::Vector3f& wi,
                                      const Eigen::Vector3f& wo) const {
  if (p_mtl.Tr)
    return Eigen::Vector3f::Constant(p_mtl.Tr);

  Eigen::Vector3f halfway = (wi + wo).normalized();
  float cos_h = std::max(0.0F, n.dot(halfway));
  float pow_cos_h = std::pow(cos_h, p_mtl.Ns);

  Eigen::Vector3f diffusion = p_mtl.Kd / M_PI;
  Eigen::Vector3f specular = p_mtl.Ks * pow_cos_h;
  return diffusion + specular;
}

}  // namespace mcpt
