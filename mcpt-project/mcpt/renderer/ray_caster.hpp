#pragma once

#include <functional>
#include <limits>

#include <Eigen/Eigen>

#include "mcpt/common/geometry/bvh_tree.hpp"
#include "mcpt/common/geometry/intersect.hpp"

namespace mcpt {

class RayCaster {
public:
  struct Intersection {
    float traveling_length = std::numeric_limits<float>::max();
    float abs_cos_incident = 0.0F;
    Eigen::Vector3f point{Eigen::Vector3f::Zero()};
    const BVHNode<float>* node = nullptr;
  };

  explicit RayCaster(const BVHTree<float>& bvh_tree)
      : RayCaster(bvh_tree, Eigen::NumTraits<float>::dummy_precision()) {}

  RayCaster(const BVHTree<float>& bvh_tree, float prec) : m_bvh_tree(bvh_tree), m_intersect(prec) {}

  Intersection Run(const Ray<float>& ray) const;

private:
  std::reference_wrapper<const BVHTree<float>> m_bvh_tree;
  Intersect<float> m_intersect;
};

}  // namespace mcpt
