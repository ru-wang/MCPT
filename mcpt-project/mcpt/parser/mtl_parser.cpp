#include "mcpt/parser/mtl_parser.hpp"

#include <fstream>
#include <sstream>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>

#include "mcpt/common/assert.hpp"

#include "mcpt/parser/misc.hpp"

namespace mcpt {

namespace {

std::vector<std::pair<std::string, Material>> ParseMaterialFile(
    const std::filesystem::path& filepath) {
  std::vector<std::pair<std::string, Material>> materials;

  std::ifstream ifs(filepath);
  while (ifs && !ifs.eof()) {
    std::stringstream linestream = SafelyGetLineStream(ifs);
    spdlog::debug("parsing {}: `{}'", filepath, linestream.str());

    std::string identifier;
    linestream >> identifier;

    if (identifier.empty() || identifier.front() == '#') {
      continue;
    } else if (identifier == "newmtl") {
      std::string mtl_name;
      linestream >> mtl_name;
      materials.emplace_back(mtl_name, Material{});
      continue;
    }

    ASSERT(!materials.empty(), "failed to parse {}: must declare material name first", filepath);
    auto& last_mtl = materials.back().second;
    if (identifier == "illum")
      linestream >> last_mtl.illum;
    else if (identifier == "Kd")
      linestream >> last_mtl.Kd[0] >> last_mtl.Kd[1] >> last_mtl.Kd[2];
    else if (identifier == "Ka")
      linestream >> last_mtl.Ka[0] >> last_mtl.Ka[1] >> last_mtl.Ka[2];
    else if (identifier == "Ks")
      linestream >> last_mtl.Ks[0] >> last_mtl.Ks[1] >> last_mtl.Ks[2];
    else if (identifier == "Ns")
      linestream >> last_mtl.Ns;
    else if (identifier == "Tr")
      linestream >> last_mtl.Tr;
    else if (identifier == "Ni")
      linestream >> last_mtl.Ni;
    else
      ASSERT_FAIL("failed to parse {}: unknown identifier `{}'", filepath, identifier);
  }

  return materials;
}

}  // namespace

MtlParser::MtlParser(const std::filesystem::path& filepath) {
  for (auto& [mtl_name, mtl] : ParseMaterialFile(filepath)) {
    auto inserted = m_materials.emplace(mtl_name, std::move(mtl)).second;
    ASSERT(inserted, "duplicate material name `{}' in {}", mtl_name, filepath);
  }
}

}  // namespace mcpt
