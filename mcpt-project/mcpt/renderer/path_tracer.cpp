#include "mcpt/renderer/path_tracer.hpp"

#include <cmath>
#include <algorithm>
#include <any>

#include "mcpt/common/assert.hpp"

#include "mcpt/common/object/mesh.hpp"

namespace mcpt {

std::optional<PathTracer::ReversePath> PathTracer::Run(const Ray<float>& incident_ray) {
  auto intersection = m_ray_caster.Run(incident_ray);
  // not intersected
  if (intersection.node == nullptr)
    return std::nullopt;

  auto& mesh = std::any_cast<const Mesh&>(intersection.node->mesh);
  auto& mtl = m_associated_object.get().GetMaterialByName(mesh.material);

  // sample a new direction
  auto [exit_dir, exit_normal, exit_pdf] = NextDirection(incident_ray.direction, mesh.normal, mtl);
  DASSERT(exit_pdf > 0.0);
  return ReversePath{
      mtl, intersection.distance, intersection.point, exit_normal, exit_dir, exit_pdf};
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
  float cos_incident = incident.dot(normal);
  DASSERT(cos_incident != 0.0F, "incident parallel to the mesh should have been rejected");

  if (material.Tr != 0.0) {
    // refraction
    // cos < 0: entering a transparent object
    // cos > 0: leaving a transparent object
    float k = cos_incident < 0.0F ? 1.0F / material.Ni : material.Ni;
    Eigen::Vector3f refract = k * incident + (1.0F - k) * cos_incident * normal;
    return {refract.normalized(), cos_incident < 0.0F ? -normal : normal, 1.0};
  } else if (material.Kd == Eigen::Vector3f::Zero() && material.Ks != Eigen::Vector3f::Zero()) {
    // ideal reflection
    Eigen::Vector3f reflect = incident - 2.0F * cos_incident * normal;
    return {reflect.normalized(), cos_incident < 0.0F ? normal : -normal, 1.0};
  } else {
    // fully opaque
    return SampleDirection(cos_incident < 0.0F ? normal : -normal, material.Ns + 1.0F);
  }
}

PathTracer::sample PathTracer::SampleDirection(const Eigen::Vector3f& normal, float alpha) {
  auto [azimuth, depression, pdf] = m_cos_pow_hemi.Random(alpha);
  DASSERT(depression < M_PI_2, "depression can not reach pi/2");

  Eigen::Vector3f dir(std::sin(depression) * std::cos(azimuth),
                      std::sin(depression) * std::sin(azimuth),
                      std::cos(depression));

  Eigen::Index x;
  normal.cwiseAbs().minCoeff(&x);

  // find the plane coordinate system
  Eigen::Matrix3f axes;
  axes.col(2) = normal.normalized();
  axes.col(1) = axes.col(2).cross(Eigen::Vector3f::Unit(x)).normalized();
  axes.col(0) = axes.col(1).cross(axes.col(2)).normalized();

  return {axes * dir, normal, pdf};
}

}  // namespace mcpt
