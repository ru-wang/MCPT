#include "mcpt/renderer/ray_caster.hpp"

#include <cmath>
#include <any>
#include <deque>

namespace mcpt {

namespace {

// threshold for rejecting self when doing intersection test
constexpr float MIN_PROJECTION_LENGTH = 0.001F;

}  // namespace

RayCaster::Intersection RayCaster::Run(const Ray<float>& ray) const {
  Intersection ret;
  float max_abs_cos_incident = 0.0F;

  // compute intersection with all the meshes and select the closest one
  for (std::deque queue{m_bvh_tree.get().root.get()}; !queue.empty(); queue.pop_front()) {
    auto node = queue.front();
    if (!m_intersect.Test(ray, node->aabb))
      continue;
    if (node->l_child)
      queue.push_back(node->l_child.get());
    if (node->r_child)
      queue.push_back(node->r_child.get());

    // is leaf node
    if (node->mesh.has_value()) {
      const Mesh& mesh = std::any_cast<std::reference_wrapper<const Mesh>>(node->mesh);

      // no intersection
      Eigen::Vector4f point_h = m_intersect.Get(ray, mesh.polygon);
      if (point_h.w() == 0.0F)
        continue;

      // reject self
      Eigen::Vector3f segment = point_h.head<3>() - ray.point_a;
      if (std::abs(segment.dot(mesh.normal)) <= MIN_PROJECTION_LENGTH)
        continue;

      // reject farther mesh
      float distance = segment.norm();
      if (distance > ret.distance)
        continue;

      // take closer mesh
      float abs_cos_incident = std::abs(ray.direction.dot(mesh.normal));
      if (distance < ret.distance || abs_cos_incident > max_abs_cos_incident) {
        max_abs_cos_incident = abs_cos_incident;
        ret.distance = distance;
        ret.point = point_h.head<3>();
        ret.node = node;
      }
    }
  }

  return ret;
}

bool RayCaster::FastCheckOcclusion(const Ray<float>& ray, float length, const Mesh& target) const {
  // compute intersection with all the meshes and select the closest one
  for (std::deque queue{m_bvh_tree.get().root.get()}; !queue.empty(); queue.pop_front()) {
    auto node = queue.front();
    if (!m_intersect.Test(ray, node->aabb))
      continue;
    if (node->l_child)
      queue.push_back(node->l_child.get());
    if (node->r_child)
      queue.push_back(node->r_child.get());

    // is leaf node
    if (node->mesh.has_value()) {
      const Mesh& mesh = std::any_cast<std::reference_wrapper<const Mesh>>(node->mesh);
      // reject target mesh
      if (&mesh == &target)
        continue;

      // no intersection
      Eigen::Vector4f point_h = m_intersect.Get(ray, mesh.polygon);
      if (point_h.w() == 0.0F)
        continue;

      // reject self
      Eigen::Vector3f segment = point_h.head<3>() - ray.point_a;
      if (std::abs(segment.dot(mesh.normal)) <= MIN_PROJECTION_LENGTH)
        continue;

      if (segment.norm() <= length - MIN_PROJECTION_LENGTH)
        return true;
    }
  }

  return false;
}

}  // namespace mcpt
