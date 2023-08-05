#include "mcpt/renderer/light_sampler.hpp"

#include <spdlog/spdlog.h>

#include "mcpt/common/assert.hpp"

namespace mcpt {

LightSampler::LightSampler(const Object& object, const BVHTree<float>& bvh_tree)
    : m_associated_object(object), m_ray_caster(bvh_tree) {
  for (const Mesh& mesh : object.light_sources()) {
    const auto& mesh_vs = mesh.polygon.vertices;
    DASSERT(mesh_vs.size() >= 3);
    for (size_t i = 1; i + 1 < mesh_vs.size(); ++i) {
      ConvexPolygon<float> tri(mesh_vs[0], mesh_vs[i], mesh_vs[i + 1]);
      float area = (mesh_vs[i] - mesh_vs[0]).cross(mesh_vs[i + 1] - mesh_vs[0]).norm() / 2.0F;
      float accum_area = m_mesh_lights.empty() ? area : m_mesh_lights.back().accum_area + area;
      m_mesh_lights.push_back({mesh, tri, area, accum_area});
    }
  }

  if (m_mesh_lights.empty())
    spdlog::warn("no light source mesh in the scene");
}

std::optional<LightSampler::PathToLight> LightSampler::Run(const Eigen::Vector3f& incident_point,
                                                           const Eigen::Vector3f& incident_normal) {
  // threshold for rejecting self and too close light sources
  static constexpr float MIN_PROJECTION_LENGTH = 0.001F;

  if (m_mesh_lights.empty())
    return std::nullopt;

  double hit_pdf = 1.0 / m_mesh_lights.back().accum_area;
  DASSERT(hit_pdf > 0.0);

  // first select a triangle
  float area = m_uni_area.Random() * m_mesh_lights.back().accum_area;
  size_t sel = 0;
  while (sel < m_mesh_lights.size() && area >= m_mesh_lights[sel].accum_area)
    ++sel;
  DASSERT(sel < m_mesh_lights.size());

  // sample a vertex in the selected triangle
  const auto& vs = m_mesh_lights[sel].triangle.vertices;
  Eigen::Vector3f hit_point(m_uni_tri.Random(vs[0], vs[1], vs[2]).data());
  Eigen::Vector3f hit_path = hit_point - incident_point;

  Ray<float> hit_ray(incident_point, hit_path);

  // check light visibility
  const auto& mesh = m_mesh_lights[sel].mesh.get();
  const auto& normal = mesh.normal;
  if (incident_normal.dot(hit_path) <= MIN_PROJECTION_LENGTH)
    return std::nullopt;
  if (normal.dot(-hit_path) <= MIN_PROJECTION_LENGTH)
    return std::nullopt;
  // check if light is blocked by other meshes
  if (m_ray_caster.FastCheckOcclusion(hit_ray, hit_path.norm(), mesh))
    return std::nullopt;

  const auto& mtl = m_associated_object.get().GetMaterialByName(mesh.material);
  return PathToLight{mtl, hit_point, normal, hit_ray.direction, hit_pdf};
}

}  // namespace mcpt
