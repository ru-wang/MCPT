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
  auto& mtl = m_ray_caster.GetAssociatedObject().GetMaterialByName(mesh.material);

  // sample a new direction
  auto [exit_dir, exit_normal, exit_pdf] = NextDirection(incident_ray.direction, mesh.normal, mtl);
  if (exit_pdf == 0.0)
    return std::nullopt;
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
  float cos_incident = std::clamp(-incident.dot(normal), -1.0F, 1.0F);
  ASSERT(cos_incident != 0.0F, "incident parallel to the mesh should have been rejected");

  if (material.Tr == 0) {
    // fully opaque
    return SampleDirection(cos_incident > 0.0F ? normal : -normal, material.Ns);
  } else {
    // transparent
    if (cos_incident > 0.0F) {
      // entering a transparent object
      if (cos_incident == 1.0F) {
        return {-normal, normal, 1.0};
      } else {
        Eigen::Vector3f perpend_n = (incident + cos_incident * normal).normalized();
        float sin_refract = std::sqrt(1.0F - cos_incident * cos_incident) / material.Ni;
        float tan_refract = sin_refract / std::sqrt(1.0F - sin_refract * sin_refract);
        return {(tan_refract * perpend_n - normal).normalized(), normal, 1.0};
      }
    } else {
      // leaving a transparent object
      return SampleDirection(normal, material.Ns);
    }
  }
}

PathTracer::sample PathTracer::SampleDirection(const Eigen::Vector3f& normal, float alpha) {
  auto [azimuth, depression, pdf] = alpha ? m_cos_pow_hemi.Random(alpha) : m_cos_hemi.Random();
  ASSERT(depression < M_PI_2, "depression can not reach pi/2");

  Eigen::Vector3f dir(std::sin(depression) * std::cos(azimuth),
                      std::sin(depression) * std::sin(azimuth),
                      std::cos(depression));

  Eigen::Index z;
  normal.maxCoeff(&z);
  Eigen::Index x = (z == 2) ? 0 : z + 1;

  // find the plane coordinate system
  Eigen::Matrix3f axes;
  axes.col(2) = normal.normalized();
  axes.col(1) = axes.col(2).cross(Eigen::Vector3f::Unit(x)).normalized();
  axes.col(0) = axes.col(1).cross(axes.col(2)).normalized();

  return {axes * dir, normal, pdf};
}

}  // namespace mcpt
