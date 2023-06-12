#pragma once

#include <filesystem>

#include "mcpt/common/object/object.hpp"

namespace mcpt::obj_parser {

class Parser {
public:
  explicit Parser(const std::filesystem::path& filepath);

  auto& object() const noexcept { return m_object; }
  auto& object() noexcept { return m_object; }

private:
  Object m_object;
};

}  // namespace mcpt::obj_parser
