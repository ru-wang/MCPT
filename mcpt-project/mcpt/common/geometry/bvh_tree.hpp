#pragma once

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "mcpt/common/assert.hpp"

#include "mcpt/common/geometry/aabb.hpp"
#include "mcpt/common/geometry/bvh_node.hpp"

namespace mcpt {

template <typename T>
struct BVHTree {
  using Scalar = T;

  std::unique_ptr<BVHNode<T>> root;

  template <typename InputIt>
  void Construct(InputIt first, InputIt last);
};

template <typename T>
template <typename InputIt>
void BVHTree<T>::Construct(InputIt first, InputIt last) {
  ASSERT(first != last);

  std::vector<std::unique_ptr<BVHNode<T>>> leaves;
  for (; first != last; ++first) {
    AABB<T> polygon_aabb;
    for (const auto& v : first->polygon.vertices)
      polygon_aabb.Update(v);
    polygon_aabb.Finish();

    auto node = std::make_unique<BVHNode<T>>(polygon_aabb);
    node->mesh = *first;
    leaves.push_back(std::move(node));
  }

  AABB<T> root_aabb;
  for (const auto& leaf : leaves)
    root_aabb.Update(leaf->aabb);
  root_aabb.Finish();

  root = std::make_unique<BVHNode<T>>(root_aabb);
  root->Split(leaves.begin(), leaves.end());
  ASSERT(root, "invalid object");
}

}  // namespace mcpt
