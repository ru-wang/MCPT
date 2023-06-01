#include "mcpt/parser/obj_parser.hpp"

#include <fstream>
#include <utility>

#include "mcpt/parser/obj_line_parser.hpp"

namespace mcpt {

ObjParser::ObjParser(const std::filesystem::path& filepath) {
  LineParser parser(LineParser::Context{filepath});

  std::ifstream ifs(filepath);
  while (ifs && !ifs.eof())
    parser.Advance(ifs);

  m_object.materials().swap(parser.ctx().materials);
  for (auto& named_mesh_group : parser.ctx().mesh_groups)
    m_object.mesh_groups().emplace_back(std::move(named_mesh_group.second));
  m_object.vertices().swap(parser.ctx().default_vertices);
  m_object.text_coords().swap(parser.ctx().default_text_coords);
  m_object.normals().swap(parser.ctx().default_normals);
}

}  // namespace mcpt
