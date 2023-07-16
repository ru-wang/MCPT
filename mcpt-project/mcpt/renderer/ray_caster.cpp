#include "mcpt/renderer/ray_caster.hpp"

#include <cmath>
#include <any>
#include <deque>

#include "mcpt/common/object/mesh.hpp"

namespace mcpt {

RayCaster::Intersection RayCaster::Run(const Ray<float>& ray) const {
  Intersection ret;

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
      auto& mesh = std::any_cast<const Mesh&>(node->mesh);

      // no intersection
      Eigen::Vector4f point_h = m_intersect.Get(ray, mesh.polygon);
      if (point_h.w() == 0.0F)
        continue;

      // reject farther mesh
      float traveling_length = (point_h.head<3>() - ray.point_a).norm();
      if (traveling_length > ret.traveling_length)
        continue;

      // take closer mesh
      float abs_cos_incident = std::abs(ray.direction.dot(mesh.normal));
      if (traveling_length < ret.traveling_length || abs_cos_incident > ret.abs_cos_incident) {
        ret.abs_cos_incident = abs_cos_incident;
        ret.traveling_length = traveling_length;
        ret.point = point_h.head<3>();
        ret.node = node;
      }
    }
  }

  return ret;
}

}  // namespace mcpt
