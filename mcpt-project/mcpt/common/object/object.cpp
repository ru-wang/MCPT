#include "mcpt/common/object/object.hpp"

#include <limits>

#include "mcpt/common/assert.hpp"
#include "mcpt/common/geometry/bvh_tree.hpp"

namespace mcpt {

namespace {

const Eigen::Vector3f MAX_VERTEX = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
const Eigen::Vector3f MIN_VERTEX = Eigen::Vector3f::Constant(std::numeric_limits<float>::lowest());

}  // namespace

const Material& Object::GetMaterialByName(const std::string& name) const {
  return m_materials.at(name);
}

const BVHNode& Object::GetBVHTree() const {
  return *m_bvh_root;
}

void Object::ConstructBVH() {
  Eigen::Vector3f object_v_max = MIN_VERTEX;
  Eigen::Vector3f object_v_min = MAX_VERTEX;

  // construct all leaves
  std::vector<std::shared_ptr<BVHNode>> bvh_leaves;
  for (const auto& mesh_group : m_mesh_groups) {
    const auto& material = mesh_group.material;

    for (const auto& mesh_index : mesh_group.mesh_index) {
      std::vector<Eigen::Vector3f> vertices;
      for (size_t vindex : mesh_index.vindex)
        vertices.push_back(m_vertices.at(vindex));

      Eigen::Vector3f mesh_v_max = MIN_VERTEX;
      Eigen::Vector3f mesh_v_min = MAX_VERTEX;
      for (const auto& v : vertices) {
        mesh_v_max = (v.array() > mesh_v_max.array()).select(v, mesh_v_max);
        mesh_v_min = (v.array() < mesh_v_min.array()).select(v, mesh_v_min);
      }

      Eigen::Vector3f avg_normal = Eigen::Vector3f::Zero();
      for (size_t nindex : mesh_index.nindex)
        avg_normal += m_normals.at(nindex);
      avg_normal /= mesh_index.nindex.size();

      auto node = std::make_shared<BVHNode>(mesh_v_min, mesh_v_max);
      node->mesh = Mesh{material, {vertices.cbegin(), vertices.cend()}, avg_normal};

      bvh_leaves.push_back(std::move(node));

      object_v_max = (mesh_v_max.array() > object_v_max.array()).select(mesh_v_max, object_v_max);
      object_v_min = (mesh_v_min.array() < object_v_min.array()).select(mesh_v_min, object_v_min);
    }
  }

  // construct the root
  m_bvh_root = std::make_shared<BVHNode>(object_v_min, object_v_max);
  BVHNode::SplitTree(m_bvh_root, bvh_leaves.begin(), bvh_leaves.end());
  ASSERT(m_bvh_root, "invalid object");
}

}  // namespace mcpt
