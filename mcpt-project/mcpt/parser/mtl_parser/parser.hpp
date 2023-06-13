#pragma once

#include <filesystem>
#include <string>
#include <unordered_map>

#include "mcpt/common/object/material.hpp"

#include "mcpt/parser/mtl_parser/context.hpp"

namespace mcpt::mtl_parser {

class Parser {
public:
  explicit Parser(const std::filesystem::path& filepath);

  auto& materials() const noexcept { return m_materials; }
  auto& materials() noexcept { return m_materials; }

private:
  void Advance(const std::string& statement, Context& ctx);

  std::unordered_map<std::string, Material> m_materials;
};

}  // namespace mcpt::mtl_parser
