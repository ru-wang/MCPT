#pragma once

#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Eigen>

#include "mcpt/common/object/material.hpp"
#include "mcpt/common/object/mesh.hpp"

namespace mcpt::obj_parser {

struct Context {
  using GroupBinding = MeshIndexGroup*;

  std::filesystem::path filepath;
  size_t linenum = 0;

  std::unordered_map<std::string, Material> materials;

  std::vector<Eigen::Vector3f> default_vertices;
  std::vector<Eigen::Vector2f> default_text_coords;
  std::vector<Eigen::Vector3f> default_normals;

  std::unordered_map<std::string, MeshIndexGroup> mesh_groups;
  int smooth_switch = 0;

  GroupBinding bound_group = nullptr;
};

}  // namespace mcpt::obj_parser
