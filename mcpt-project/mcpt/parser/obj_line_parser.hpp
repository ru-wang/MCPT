#pragma once

#include <filesystem>
#include <istream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include <Eigen/Eigen>

#include "mcpt/common/object/material.hpp"
#include "mcpt/common/object/mesh.hpp"
#include "mcpt/common/object/object.hpp"

#include "mcpt/parser/obj_parser.hpp"

namespace mcpt {

class ObjParser::LineParser {
public:
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

  explicit LineParser(Context init_ctx);

  auto& ctx() const noexcept { return m_ctx; }
  auto& ctx() noexcept { return m_ctx; }

  void Advance(std::istream& is);
  void Advance(const std::string& statement);

private:
  void Advance(const std::string& identifier, const std::string& declaration);

  void ParseMTLFilename(const std::filesystem::path& mtl_filename);
  void ParseGroupName(const std::string& group_name);

  template <Eigen::Index Size, typename Derived>
  void ParseNumericVector(const std::string& tokens, Eigen::MatrixBase<Derived>& vector);

  void ParseMeshIndex(const std::string& tokens);
  void ParseSmoothSwitch(const std::string& smooth_decl);
  void ParseMaterialDecl(const std::string& material_name);

  Context m_ctx;
};

}  // namespace mcpt
