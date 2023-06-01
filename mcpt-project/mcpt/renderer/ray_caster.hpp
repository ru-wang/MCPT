#pragma once

#include <functional>
#include <limits>
#include <memory>

#include <Eigen/Eigen>

#include "mcpt/common/geometry/bvh_tree.hpp"
#include "mcpt/common/geometry/intersect.hpp"
#include "mcpt/common/geometry/types.hpp"
#include "mcpt/common/object/object.hpp"

namespace mcpt {

class RayCaster {
public:
  struct Intersection {
    float abs_cos_incident = 0.0F;
    float traveling_length = std::numeric_limits<float>::max();
    Eigen::Vector3f point{Eigen::Vector3f::Zero()};
    std::shared_ptr<const BVHNode> node;
  };

  explicit RayCaster(const Object& object) : m_object(object) {}
  RayCaster(const Object& object, float prec) : m_object(object), m_intersect(prec) {}

  Intersection Run(const Ray<float>& ray) const;

  const Object& GetAssociatedObject() const noexcept { return m_object.get(); }

private:
  std::reference_wrapper<const Object> m_object;
  Intersect<float> m_intersect{Eigen::NumTraits<float>::dummy_precision()};
};

}  // namespace mcpt
