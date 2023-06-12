#pragma once

#include <filesystem>
#include <istream>
#include <string>

#include <Eigen/Eigen>

#include "mcpt/parser/obj_parser/context.hpp"

namespace mcpt::obj_parser {

class LineParser {
public:
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

}  // namespace mcpt::obj_parser
