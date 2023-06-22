#include "mcpt/common/object/object.hpp"

#include <functional>

namespace mcpt {

const Material& Object::GetMaterialByName(const std::string& name) const {
  return m_materials.at(name);
}

BVHTree<float> Object::CreateBVHTree() const {
  // construct all meshes
  std::vector<Mesh> meshes;
  for (const auto& [material, mesh_index] : m_mesh_groups) {
    for (const auto& [vindex, tindex, nindex] : mesh_index) {
      std::vector<Eigen::Vector3f> vertices;
      for (size_t index : vindex)
        vertices.push_back(m_vertices.at(index));

      std::vector<Eigen::Vector2f> text_coords;
      for (size_t index : vindex)
        text_coords.push_back(m_text_coords.at(index));

      Eigen::Vector3f avg_normal = Eigen::Vector3f::Zero();
      for (size_t index : nindex)
        avg_normal += m_normals.at(index);
      avg_normal /= nindex.size();

      meshes.push_back({material, ConvexPolygon{vertices}, Polygon2D{text_coords}, avg_normal});
    }
  }

  BVHTree<float> bvh_tree;
  bvh_tree.Construct(meshes.cbegin(), meshes.cend());
  return bvh_tree;
}

}  // namespace mcpt
