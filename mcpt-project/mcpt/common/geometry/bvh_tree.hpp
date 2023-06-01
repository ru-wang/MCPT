#pragma once

#include <any>
#include <iterator>
#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Eigen>

#include "mcpt/common/assert.hpp"

#include "mcpt/common/geometry/aabb.hpp"

namespace mcpt {

// bounding volume hierarchies class
// represents a node in the binary bvh tree
struct BVHNode : std::enable_shared_from_this<BVHNode> {
  // axis-aligned bounding box for current level (left child and right child)
  AABB<float> aabb;

  // only leaf nodes have corresponding mesh
  std::any mesh;

  std::shared_ptr<BVHNode> l_child;
  std::shared_ptr<BVHNode> r_child;

  template <typename T, typename U>
  BVHNode(const Eigen::MatrixBase<T>& min_vertex, const Eigen::MatrixBase<U>& max_vertex)
      : aabb(min_vertex, max_vertex) {}

  template <typename InputIt>
  static void SplitTree(std::shared_ptr<BVHNode>& parent, InputIt first, InputIt last);
};

template <typename InputIt>
void BVHNode::SplitTree(std::shared_ptr<BVHNode>& parent, InputIt first, InputIt last) {
  // can not be splitted
  if (first == last) {
    parent.reset();
    return;
  }

  // leaf node not be splitted any more
  if (std::distance(first, last) == 1) {
    parent = std::move(*first);
    return;
  }

  Eigen::Index max_coeff;
  parent->aabb.diagonal().maxCoeff(&max_coeff);

  // find a hyperplane in P3 space splitting the space into two partitions
  Eigen::Vector4f hyperplane = Eigen::Vector4f::Zero();
  switch (max_coeff) {
    case 0: {
      hyperplane.x() = 1.0F;
      hyperplane.w() = -parent->aabb.center().x();

      Eigen::Vector3f new_v_min = parent->aabb.min_vertex();
      Eigen::Vector3f new_v_max = parent->aabb.max_vertex();
      new_v_min.x() = parent->aabb.center().x();
      new_v_max.x() = parent->aabb.center().x();

      parent->l_child = std::make_unique<BVHNode>(parent->aabb.min_vertex(), new_v_max);
      parent->r_child = std::make_unique<BVHNode>(new_v_min, parent->aabb.max_vertex());
    } break;

    case 1: {
      hyperplane.y() = 1.0F;
      hyperplane.w() = -parent->aabb.center().y();

      Eigen::Vector3f new_v_min = parent->aabb.min_vertex();
      Eigen::Vector3f new_v_max = parent->aabb.max_vertex();
      new_v_min.y() = parent->aabb.center().y();
      new_v_max.y() = parent->aabb.center().y();

      parent->l_child = std::make_unique<BVHNode>(parent->aabb.min_vertex(), new_v_max);
      parent->r_child = std::make_unique<BVHNode>(new_v_min, parent->aabb.max_vertex());
    } break;

    case 2: {
      hyperplane.z() = 1.0F;
      hyperplane.w() = -parent->aabb.center().z();

      Eigen::Vector3f new_v_min = parent->aabb.min_vertex();
      Eigen::Vector3f new_v_max = parent->aabb.max_vertex();
      new_v_min.z() = parent->aabb.center().z();
      new_v_max.z() = parent->aabb.center().z();

      parent->l_child = std::make_unique<BVHNode>(parent->aabb.min_vertex(), new_v_max);
      parent->r_child = std::make_unique<BVHNode>(new_v_min, parent->aabb.max_vertex());
    } break;

    default:
      // this should not happen
      ASSERT_FAIL();
      break;
  }

  std::vector<std::shared_ptr<BVHNode>> l_leaves;
  std::vector<std::shared_ptr<BVHNode>> r_leaves;

  for (; first != last; ++first) {
    std::shared_ptr<BVHNode>& leaf = *first;
    parent->aabb.UpdateMin(leaf->aabb.min_vertex());
    parent->aabb.UpdateMax(leaf->aabb.max_vertex());

    if (leaf->aabb.center().homogeneous().dot(hyperplane) < 0) {
      l_leaves.push_back(std::move(leaf));
    } else if (leaf->aabb.center().homogeneous().dot(hyperplane) > 0) {
      r_leaves.push_back(std::move(leaf));
    } else {
      if (l_leaves.size() <= r_leaves.size())
        l_leaves.push_back(std::move(leaf));
      else
        r_leaves.push_back(std::move(leaf));
    }
  }
  parent->aabb.UpdateCenterDiag();

  SplitTree(parent->l_child, l_leaves.begin(), l_leaves.end());
  SplitTree(parent->r_child, r_leaves.begin(), r_leaves.end());
}

}  // namespace mcpt
