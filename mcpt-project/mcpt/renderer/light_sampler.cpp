#include "mcpt/renderer/light_sampler.hpp"

#include <spdlog/spdlog.h>

#include "mcpt/common/assert.hpp"

namespace mcpt {

namespace {

constexpr bool CosineCloseToPlusMinus1(float val, float epsilon) {
  return val >= 1.0F - epsilon || val <= -1.0F + epsilon;
}

}  // namespace

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
  // threshold for rejecting self or invisible light sources or small projected light sources
  static constexpr float COSINE_EPSILON = 0.001F;

  if (m_mesh_lights.empty())
    return std::nullopt;

  // first select a triangle
  float area = m_uni_area.Random() * m_mesh_lights.back().accum_area;
  size_t sel = 0;
  while (sel < m_mesh_lights.size() && area >= m_mesh_lights[sel].accum_area)
    ++sel;
  DASSERT(sel < m_mesh_lights.size());
  const auto& light = m_mesh_lights[sel];

  // sample a vertex in the selected triangle
  auto sample = SampleSphericalTriangle(incident_point, light.triangle);
  if (!sample.has_value())
    return std::nullopt;
  auto [hit_dir, hit_pdf] = sample.value();
  hit_pdf *= light.area / m_mesh_lights.back().accum_area;

  // check light visibility
  const auto& mesh = light.mesh.get();
  const auto& normal = mesh.normal;
  if (incident_normal.dot(hit_dir) <= COSINE_EPSILON)
    return std::nullopt;
  if (normal.dot(-hit_dir) <= COSINE_EPSILON)
    return std::nullopt;

  // check if light is blocked by other meshes
  Eigen::Vector4f inter_p = m_ray_caster.IsVisible(Ray<float>(incident_point, hit_dir), mesh);
  if (inter_p.w() == 0.0F)
    return std::nullopt;

  const auto& mtl = m_associated_object.get().GetMaterialByName(mesh.material);
  return PathToLight{mtl, inter_p.head<3>(), normal, hit_dir, hit_pdf};
}

std::optional<Direction<float>> LightSampler::SampleSphericalTriangle(
    const Eigen::Vector3f& center, const ConvexPolygon<float>& tri) {
  // threshold for rejecting close light sources
  static constexpr float LENGTH_EPSILON = 0.001F;
  // threshold for rejecting small spherical triangle areas
  static constexpr float COSINE_EPSILON = 1.0e-07F;

  // project the triangle onto the unit sphere
  Eigen::Vector3f va = tri.vertices[0] - center;
  Eigen::Vector3f vb = tri.vertices[1] - center;
  Eigen::Vector3f vc = tri.vertices[2] - center;

  if (va.lpNorm<Eigen::Infinity>() <= LENGTH_EPSILON ||
      vb.lpNorm<Eigen::Infinity>() <= LENGTH_EPSILON ||
      vc.lpNorm<Eigen::Infinity>() <= LENGTH_EPSILON)
    return std::nullopt;

  va.normalize();
  vb.normalize();
  vc.normalize();

  Eigen::Vector3f na = vb.cross(vc).normalized();
  Eigen::Vector3f nb = vc.cross(va).normalized();
  Eigen::Vector3f nc = va.cross(vb).normalized();

  float cos_alpha = -nb.dot(nc);
  float cos_beta = -na.dot(nc);
  float cos_gamma = -na.dot(nb);

  if (CosineCloseToPlusMinus1(cos_alpha, COSINE_EPSILON) ||
      CosineCloseToPlusMinus1(cos_beta, COSINE_EPSILON) ||
      CosineCloseToPlusMinus1(cos_gamma, COSINE_EPSILON))
    return std::nullopt;
  return m_uni_ust.Random(va, vb, vc, cos_alpha, cos_beta, cos_gamma);
}

}  // namespace mcpt
