#pragma once

#include <filesystem>
#include <string>
#include <unordered_map>

#include "mcpt/common/object/material.hpp"

namespace mcpt::mtl_parser {

struct Context {
  using AssociatedMaterial = Material*;

  std::filesystem::path filepath;
  size_t linenum = 0;

  std::unordered_map<std::string, Material> materials;

  AssociatedMaterial associated_material = nullptr;
};

}  // namespace mcpt::mtl_parser
