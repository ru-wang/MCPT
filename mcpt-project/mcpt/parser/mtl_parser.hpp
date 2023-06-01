#pragma once

#include <filesystem>
#include <string>
#include <unordered_map>

#include "mcpt/common/object/material.hpp"

namespace mcpt {

class MtlParser {
public:
  explicit MtlParser(const std::filesystem::path& filepath);

  auto& materials() const noexcept { return m_materials; }
  auto& materials() noexcept { return m_materials; }

private:
  std::unordered_map<std::string, Material> m_materials;
};

}  // namespace mcpt
