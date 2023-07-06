#pragma once

#include <algorithm>
#include <any>
#include <iterator>
#include <memory>
#include <utility>

#include <Eigen/Eigen>

#include "mcpt/common/assert.hpp"

#include "mcpt/common/geometry/aabb.hpp"

namespace mcpt {

// bounding volume hierarchies class
// represents a node in the binary bvh tree
template <typename T>
struct BVHNode {
  using Scalar = T;

  // axis-aligned bounding box for current level (left child and right child)
  AABB<T> aabb;

  // only leaf nodes have corresponding mesh (cref)
  std::any mesh;

  std::unique_ptr<BVHNode> l_child;
  std::unique_ptr<BVHNode> r_child;

  BVHNode() = default;
  explicit BVHNode(const AABB<T>& aabb) : aabb(aabb) {}

  template <typename InputIt>
  void Split(InputIt first, InputIt last);
};

template <typename T>
template <typename InputIt>
void BVHNode<T>::Split(InputIt first, InputIt last) {
  size_t n = std::distance(first, last);
  size_t l = n / 2;
  ASSERT(n > 1);

  // split the space into two partitions
  Eigen::Index sort_axis;
  aabb.diagonal().maxCoeff(&sort_axis);
  std::nth_element(first, first + l, last, [sort_axis](const auto& lhs, const auto& rhs) {
    if (lhs->aabb.min_vertex().coeff(sort_axis) > rhs->aabb.min_vertex().coeff(sort_axis))
      return false;
    return lhs->aabb.min_vertex().coeff(sort_axis) < rhs->aabb.min_vertex().coeff(sort_axis) ||
           lhs->aabb.max_vertex().coeff(sort_axis) < rhs->aabb.max_vertex().coeff(sort_axis);
  });

  if (l == 1) {
    l_child = std::move(*first);
  } else {
    AABB<T> parent_aabb;
    for (auto it = first; it != first + l; ++it)
      parent_aabb.Update((*it)->aabb);
    parent_aabb.Finish();

    l_child = std::make_unique<BVHNode>(parent_aabb);
    l_child->Split(first, first + l);
  }

  if (n - l == 1) {
    r_child = std::move(*(first + l));
  } else {
    AABB<T> parent_aabb;
    for (auto it = first + l; it != last; ++it)
      parent_aabb.Update((*it)->aabb);
    parent_aabb.Finish();

    r_child = std::make_unique<BVHNode>(parent_aabb);
    r_child->Split(first + l, last);
  }
}

}  // namespace mcpt
