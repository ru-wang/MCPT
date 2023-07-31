#include "mcpt/common/object/object.hpp"

#include <functional>
#include <limits>

#include <spdlog/spdlog.h>

namespace mcpt {

const Material& Object::GetMaterialByName(const std::string& name) const {
  return m_materials.at(name);
}

BVHTree<float> Object::CreateBVHTree() {
  spdlog::info("construct BVH tree from object:");
  spdlog::info("  #vertex: {}", m_vertices.size());
  spdlog::info("  #texture coordinate: {}", m_text_coords.size());
  spdlog::info("  #normal: {}", m_normals.size());
  spdlog::info("  #mesh group: {}", m_mesh_groups.size());

  size_t num_meshes = 0;

  Eigen::Vector3f min_mesh_vertex = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
  Eigen::Vector3f max_mesh_vertex = Eigen::Vector3f::Constant(std::numeric_limits<float>::lowest());

  // construct all meshes
  for (const auto& [material, mesh_index] : m_mesh_groups) {
    num_meshes += mesh_index.size();

    for (const auto& [vindex, tindex, nindex] : mesh_index) {
      std::vector<Eigen::Vector3f> vertices;
      for (size_t index : vindex) {
        min_mesh_vertex = min_mesh_vertex.cwiseMin(m_vertices.at(index));
        max_mesh_vertex = max_mesh_vertex.cwiseMax(m_vertices.at(index));
        vertices.push_back(m_vertices.at(index));
      }

      std::vector<Eigen::Vector2f> text_coords;
      for (size_t index : tindex)
        text_coords.push_back(m_text_coords.at(index));

      Eigen::Vector3f avg_normal = Eigen::Vector3f::Zero();
      for (size_t index : nindex)
        avg_normal += m_normals.at(index);
      avg_normal.normalize();

      m_meshes.push_back({material, ConvexPolygon{vertices}, Polygon2D{text_coords}, avg_normal});
    }
  }

  for (const auto& mesh : m_meshes) {
    if (Material::Type(GetMaterialByName(mesh.material)) == Material::EM)
      m_light_sources.emplace_back(mesh);
  }

  static const Eigen::IOFormat FMT{Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " "};

  spdlog::info("total meshes: {}", num_meshes);
  spdlog::info("  min: {}", min_mesh_vertex.format(FMT));
  spdlog::info("  max: {}", max_mesh_vertex.format(FMT));

  BVHTree<float> bvh_tree;
  bvh_tree.Construct(m_meshes.cbegin(), m_meshes.cend());

  spdlog::info("BVH tree leaves: {}", bvh_tree.num_leaves);
  spdlog::info("  min: {}", bvh_tree.root->aabb.min_vertex().format(FMT));
  spdlog::info("  max: {}", bvh_tree.root->aabb.max_vertex().format(FMT));

  return bvh_tree;
}

}  // namespace mcpt
