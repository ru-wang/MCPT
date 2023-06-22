#pragma once

#include <any>
#include <memory>
#include <utility>
#include <vector>

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

  explicit BVHNode(const AABB<T>& aabb) : aabb(aabb) {}

  template <typename InputIt>
  void Split(InputIt first, InputIt last);
};

template <typename T>
template <typename InputIt>
void BVHNode<T>::Split(InputIt first, InputIt last) {
  ASSERT(first != last);

  l_child = std::make_unique<BVHNode>(aabb);
  r_child = std::make_unique<BVHNode>(aabb);

  // find a hyperplane in P3 space splitting the space into two partitions
  Eigen::Index max_coeff;
  aabb.diagonal().maxCoeff(&max_coeff);
  Eigen::Vector4f hyperplane = Eigen::Vector4f::Unit(max_coeff);
  hyperplane.w() = -aabb.center().coeff(max_coeff);

  // split the leaves by the hyperplane
  std::vector<std::unique_ptr<BVHNode>> l_leaves;
  std::vector<std::unique_ptr<BVHNode>> r_leaves;
  for (; first != last; ++first) {
    auto& leaf = *first;
    T dist = leaf->aabb.center().homogeneous().dot(hyperplane);
    if (dist < 0.0 || (dist == 0.0 && l_leaves.size() <= r_leaves.size())) {
      l_child->aabb.Update(leaf->aabb);
      l_leaves.push_back(std::move(leaf));
    } else {
      r_child->aabb.Update(leaf->aabb);
      r_leaves.push_back(std::move(leaf));
    }
  }

  l_child->aabb.Finish();
  r_child->aabb.Finish();

  switch (l_leaves.size()) {
    case 0: l_child = nullptr; break;
    case 1: l_child = std::move(l_leaves.front()); break;
    default: l_child->Split(l_leaves.begin(), l_leaves.end());
  }
  switch (r_leaves.size()) {
    case 0: r_child = nullptr; break;
    case 1: r_child = std::move(r_leaves.front()); break;
    default: r_child->Split(r_leaves.begin(), r_leaves.end());
  }
}

}  // namespace mcpt
