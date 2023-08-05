#include "mcpt/renderer/path_tracer.hpp"

#include <cmath>
#include <any>

#include "mcpt/common/assert.hpp"
#include "mcpt/common/object/mesh.hpp"

namespace mcpt {

std::optional<PathTracer::ReversePath> PathTracer::Run(const Ray<float>& incident_ray) {
  auto intersection = m_ray_caster.Run(incident_ray);
  // not intersected
  if (intersection.node == nullptr)
    return std::nullopt;

  const Mesh& mesh = std::any_cast<std::reference_wrapper<const Mesh>>(intersection.node->mesh);
  const Material& mtl = m_associated_object.get().GetMaterialByName(mesh.material);

  // sample a new direction
  auto [exit_normal, exit_dir, exit_pdf] = NextDirection(incident_ray.direction, mesh.normal, mtl);
  return ReversePath{mtl, intersection.point, exit_normal, exit_dir, exit_pdf};
}

/**
 *             N
 * incident    |    reflection
 *         \   |   /
 *          \ _|_ /
 *          /\ | /\ diffusion
 *    _____|__\|/__|_____________ BRDF
 *             *
 *              * retraction
 *  transparent *
 *               *
 *               *
 *    ____________*______________ BTDF
 *             |  |\  |
 *              \_|_\/ diffusion
 *                |  \
 *                |   \ retraction
 *                |
 *                N
 */
PathTracer::sample PathTracer::NextDirection(const Eigen::Vector3f& incident,
                                             const Eigen::Vector3f& normal,
                                             const Material& material) {
  DASSERT(normal.dot(incident) != 0.0F, "incident parallel to the mesh should have been rejected");
  switch (Material::Type(material)) {
    case Material::TR:
      // refraction
      return SampleRefraction(incident, normal, material.Ni);
      break;
    case Material::SPEC:
      // ideal reflection
      return SampleReflection(incident, normal);
      break;
    case Material::DIFF:
      // diffusion
      return SampleDiffusion(normal, material.Ns + 1.0F);
      break;
    default: return {normal, Eigen::Vector3f::Zero(), 0.0}; break;
  }
}

PathTracer::sample PathTracer::SampleReflection(const Eigen::Vector3f& incident,
                                                const Eigen::Vector3f& normal) {
  Eigen::Vector3f refl = incident - 2.0F * normal.dot(incident) * normal;
  return {normal, refl.normalized(), 1.0};
}

PathTracer::sample PathTracer::SampleRefraction(const Eigen::Vector3f& incident,
                                                const Eigen::Vector3f& normal,
                                                float refr_k) {
  // cos < 0: entering a transparent object
  // cos > 0: leaving a transparent object
  // cos = 0: impossible since parallel is rejected already
  float cos = normal.dot(incident);
  if (cos < 0.0F) {
    float sin2_k = (1.0F - cos * cos) / (refr_k * refr_k);
    Eigen::Vector3f refr = (incident - cos * normal) / refr_k - std::sqrt(1.0 - sin2_k) * normal;
    return {-normal, refr.normalized(), 1.0};
  } else {
    float sin2_k = (1.0F - cos * cos) * (refr_k * refr_k);
    if (sin2_k >= 1.0F) {
      // total reflection
      Eigen::Vector3f refl = incident - 2.0F * cos * normal;
      return {-normal, refl.normalized(), 1.0};
    } else {
      // refraction
      Eigen::Vector3f refr = (incident - cos * normal) * refr_k + std::sqrt(1.0 - sin2_k) * normal;
      return {normal, refr.normalized(), 1.0};
    }
  }
}

PathTracer::sample PathTracer::SampleDiffusion(const Eigen::Vector3f& normal, float alpha) {
  auto [azimuth, depression, pdf] = m_cos_pow_hemi.Random(alpha);
  DASSERT(depression < M_PI_2, "depression can not reach pi/2");

  Eigen::Vector3f dir(std::sin(depression) * std::cos(azimuth),
                      std::sin(depression) * std::sin(azimuth),
                      std::cos(depression));

  Eigen::Index x;
  normal.cwiseAbs().minCoeff(&x);

  // find the plane coordinate system
  Eigen::Matrix3f axes;
  axes.col(2) = normal;
  axes.col(1) = axes.col(2).cross(Eigen::Vector3f::Unit(x)).normalized();
  axes.col(0) = axes.col(1).cross(axes.col(2)).normalized();

  return {normal, axes * dir, pdf};
}

}  // namespace mcpt
