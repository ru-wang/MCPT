#include "mcpt/renderer/ray_caster.hpp"

#include <cmath>
#include <any>
#include <deque>

#include "mcpt/common/object/mesh.hpp"

namespace mcpt {

RayCaster::Intersection RayCaster::Run(const Ray<float>& ray) const {
  Intersection ret;

  // compute intersection with all the meshes and select the closest one
  const BVHNode* found = nullptr;
  std::deque<const BVHNode*> bfs_queue{&m_object.get().GetBVHTree()};

  while (!bfs_queue.empty()) {
    auto node = bfs_queue.front();
    bfs_queue.pop_front();

    if (!m_intersect.Test(ray, node->aabb))
      continue;
    if (node->l_child)
      bfs_queue.push_back(node->l_child.get());
    if (node->r_child)
      bfs_queue.push_back(node->r_child.get());

    // is leaf node
    if (node->mesh.has_value()) {
      auto& mesh = std::any_cast<const Mesh&>(node->mesh);

      float abs_cos_incident = std::abs(ray.direction.dot(mesh.normal));
      // reject parallel
      if (abs_cos_incident == 0.0F)
        continue;

      // no intersection
      Eigen::Vector4f point_h = m_intersect.Get(ray, mesh.polygon);
      if (point_h.w() == 0.0F)
        continue;

      // reject farther mesh
      float traveling_len = (point_h.head<3>() - ray.point_a).norm();
      if (traveling_len > ret.traveling_length)
        continue;

      // take closer mesh
      if (traveling_len < ret.traveling_length || abs_cos_incident > ret.abs_cos_incident) {
        ret.abs_cos_incident = abs_cos_incident;
        ret.traveling_length = traveling_len;
        ret.point = point_h.head<3>();
        found = node;
      }
    }
  }

  if (found)
    ret.node = found->shared_from_this();
  return ret;
}

}  // namespace mcpt
