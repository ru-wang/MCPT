#pragma once

#include <limits>

#include <Eigen/Eigen>

#include "mcpt/common/geometry/bvh_tree.hpp"
#include "mcpt/common/geometry/intersect.hpp"
#include "mcpt/common/object/object.hpp"

namespace mcpt {

class RayCaster {
public:
  struct Intersection {
    float abs_cos_incident = 0.0F;
    float traveling_length = std::numeric_limits<float>::max();
    Eigen::Vector3f point{Eigen::Vector3f::Zero()};

    const BVHNode<float>* node = nullptr;
  };

  explicit RayCaster(const Object& object)
      : RayCaster(object, Eigen::NumTraits<float>::dummy_precision()) {}

  RayCaster(const Object& object, float prec)
      : m_bvh_tree(object.CreateBVHTree()), m_intersect(prec) {}

  Intersection Run(const Ray<float>& ray) const;

private:
  BVHTree<float> m_bvh_tree;
  Intersect<float> m_intersect;
};

}  // namespace mcpt
