#include "mcpt/renderer/light_sampler.hpp"

#include <cmath>
#include <algorithm>

#include <spdlog/spdlog.h>

#include "mcpt/common/assert.hpp"
#include "mcpt/common/random.hpp"
#include "mcpt/common/random_triangle.hpp"

namespace mcpt {

namespace {
// threshold for rejecting small spherical triangle areas
constexpr float AREA_EPSILON = 0.001F;
// threshold for testing hit ray and point surface and light source surface
constexpr float COSINE_EPSILON = 0.0001F;
}  // namespace

LightSampler::LightSampler(const Object& object, const BVHTree<float>& bvh_tree)
    : m_associated_object(object), m_ray_caster(bvh_tree) {
  for (const auto& mesh : object.light_sources())
    AddTriangleLights(mesh);
  ASSERT(!m_triangle_lights.empty(), "no light source mesh in the scene");
}

std::optional<PathToLight> LightSampler::Run(const Eigen::Vector3f& start_point,
                                             const Eigen::Vector3f& start_normal) {
  // first select a triangle
  float area = Uniform<float>().Random() * m_triangle_lights.back().accum_area;
  size_t sel = 0;
  while (sel < m_triangle_lights.size() && area >= m_triangle_lights[sel].accum_area)
    ++sel;
  const auto& light = m_triangle_lights[sel];

  // sample a vertex in the selected triangle
  auto [hit_pdf, hit_point, hit_dir] = HitDirection(start_point, start_normal, light);
  if (hit_pdf == 0.0)
    return std::nullopt;
  hit_pdf *= light.area / m_triangle_lights.back().accum_area;

  const auto& mtl = m_associated_object.get().GetMaterialByName(light.mesh.get().material);
  return PathToLight{mtl, hit_point, light.mesh.get().normal, hit_dir, hit_pdf};
}

void LightSampler::AddTriangleLights(const Mesh& light) {
  const auto& verts = light.polygon.vertices;
  ASSERT(verts.size() >= 3, "invalid number of vertices: {}", verts.size());

  // split the convex polygon light into triangle fans
  for (size_t i = 1; i + 1 < verts.size(); ++i) {
    float area = (verts[i] - verts[0]).cross(verts[i + 1] - verts[0]).norm() / 2.0F;
    float accum_area = area;
    if (!m_triangle_lights.empty())
      accum_area += m_triangle_lights.back().accum_area;

    // create association between trianlge light and the original mesh
    ConvexPolygon<float> triangle(verts[0], verts[i], verts[i + 1]);
    m_triangle_lights.push_back({light, triangle, area, accum_area});
  }
}

LightSampler::sample LightSampler::HitDirection(const Eigen::Vector3f& point,
                                                const Eigen::Vector3f& normal,
                                                const TriangleLight& light) {
  // project the triangle onto the unit sphere
  Eigen::Vector3f A = light.triangle.vertices[0] - point;
  Eigen::Vector3f B = light.triangle.vertices[1] - point;
  Eigen::Vector3f C = light.triangle.vertices[2] - point;

  // internal angles of the spherical triangle
  Eigen::Vector3f n_arc_a = B.cross(C).normalized();
  Eigen::Vector3f n_arc_b = C.cross(A).normalized();
  Eigen::Vector3f n_arc_c = A.cross(B).normalized();
  float alpha = std::acos(std::clamp(-n_arc_b.dot(n_arc_c), -1.0F, 1.0F));
  float beta = std::acos(std::clamp(-n_arc_a.dot(n_arc_c), -1.0F, 1.0F));
  float gamma = std::acos(std::clamp(-n_arc_a.dot(n_arc_b), -1.0F, 1.0F));

  // each spherical internal angle is between (0,pi)
  DASSERT(
      alpha > 0.0F && beta > 0.0F && gamma > 0.0F, "invalid angles: {} {} {}", alpha, beta, gamma);

  float area = alpha + beta + gamma - M_PI;
  if (area <= AREA_EPSILON) {
    return SamplePlaneLight(point, normal, light);
  } else {
    A.normalize();
    B.normalize();
    C.normalize();
    return SampleSphericalLight(point, normal, light, A, B, C, area);
  }
}

LightSampler::sample LightSampler::SamplePlaneLight(const Eigen::Vector3f& point,
                                                    const Eigen::Vector3f& normal,
                                                    const TriangleLight& light) {
  const Mesh& light_mesh = light.mesh;

  Eigen::Vector3f hit_point = UniformTriangle<float>().Random(
      light.triangle.vertices[0], light.triangle.vertices[1], light.triangle.vertices[2]);
  Eigen::Vector3f hit_path = hit_point - point;
  Ray<float> hit_ray(point, hit_path);

  if (!IsVisible(hit_ray, light_mesh, normal, hit_path.norm())) {
    return {0.0};
  } else {
    double pdf = hit_path.squaredNorm() / light_mesh.normal.dot(-hit_ray.direction) / light.area;
    ASSERT(pdf > 0.0, "invalid PDF sampling plane triangle: {}", pdf);
    return {pdf, hit_point, hit_ray.direction};
  }
}

LightSampler::sample LightSampler::SampleSphericalLight(const Eigen::Vector3f& point,
                                                        const Eigen::Vector3f& normal,
                                                        const TriangleLight& light,
                                                        const Eigen::Vector3f& A,
                                                        const Eigen::Vector3f& B,
                                                        const Eigen::Vector3f& C,
                                                        float area) {
  const Mesh& light_mesh = light.mesh;

  Ray<float> hit_ray(point, UniformUnitSphericalTriangle<float>().Random(A, B, C));
  Eigen::Vector4f inter_p = m_ray_caster.IntersectPlane(hit_ray, light_mesh.polygon);
  if (inter_p.w() == 0.0F)
    return {0.0};
  Eigen::Vector3f hit_point = inter_p.hnormalized();

  if (!IsVisible(hit_ray, light_mesh, normal, (hit_point - point).norm())) {
    return {0.0};
  } else {
    double pdf = 1.0 / area;
    ASSERT(pdf > 0.0, "invalid PDF sampling spherical triangle: {}", pdf);
    return {pdf, hit_point, hit_ray.direction};
  }
}

bool LightSampler::IsVisible(const Ray<float>& hit_ray,
                             const Mesh& target,
                             const Eigen::Vector3f& normal,
                             float hit_dist) const {
  return (hit_ray.direction.dot(normal) > COSINE_EPSILON) &&
         (hit_ray.direction.dot(-target.normal) > COSINE_EPSILON) &&
         !m_ray_caster.IsBlocked(hit_ray, target, hit_dist);
}

}  // namespace mcpt
