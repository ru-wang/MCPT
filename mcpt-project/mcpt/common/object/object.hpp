#pragma once

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Eigen>

#include "mcpt/common/object/material.hpp"
#include "mcpt/common/object/mesh.hpp"

#include "mcpt/common/geometry/bvh_tree.hpp"

namespace mcpt {

class Object {
public:
  auto& materials() const noexcept { return m_materials; }
  auto& materials() noexcept { return m_materials; }

  auto& mesh_groups() const noexcept { return m_mesh_groups; }
  auto& mesh_groups() noexcept { return m_mesh_groups; }

  auto& vertices() const noexcept { return m_vertices; }
  auto& vertices() noexcept { return m_vertices; }

  auto& text_coords() const noexcept { return m_text_coords; }
  auto& text_coords() noexcept { return m_text_coords; }

  auto& normals() const noexcept { return m_normals; }
  auto& normals() noexcept { return m_normals; }

  auto& meshes() const noexcept { return m_meshes; }
  auto& light_sources() const noexcept { return m_light_sources; }

  const Material& GetMaterialByName(const std::string& name) const;

  // create a BVH tree and bind the current object to it
  BVHTree<float> CreateBVHTree();

private:
  std::unordered_map<std::string, Material> m_materials;
  std::vector<MeshIndexGroup> m_mesh_groups;

  std::vector<Eigen::Vector3f> m_vertices;
  std::vector<Eigen::Vector2f> m_text_coords;
  std::vector<Eigen::Vector3f> m_normals;

  std::vector<Mesh> m_meshes;
  std::vector<std::reference_wrapper<const Mesh>> m_light_sources;
};

}  // namespace mcpt
