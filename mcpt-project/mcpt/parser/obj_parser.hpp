#pragma once

#include <filesystem>

#include "mcpt/common/object/object.hpp"

namespace mcpt {

class ObjParser {
public:
  class LineParser;

  explicit ObjParser(const std::filesystem::path& filepath);

  auto& object() const noexcept { return m_object; }
  auto& object() noexcept { return m_object; }

private:
  Object m_object;
};

}  // namespace mcpt
