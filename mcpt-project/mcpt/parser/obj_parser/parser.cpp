#include "mcpt/parser/obj_parser/parser.hpp"

#include <fstream>
#include <utility>

#include "mcpt/parser/obj_parser/context.hpp"
#include "mcpt/parser/obj_parser/line_parser.hpp"

namespace mcpt::obj_parser {

Parser::Parser(const std::filesystem::path& filepath) {
  LineParser parser(Context{filepath});

  std::ifstream ifs(filepath);
  while (ifs && !ifs.eof())
    parser.Advance(ifs);

  m_object.materials() = std::move(parser.ctx().materials);
  for (auto& named_mesh_group : parser.ctx().mesh_groups)
    m_object.mesh_groups().push_back(std::move(named_mesh_group.second));
  m_object.vertices() = std::move(parser.ctx().default_vertices);
  m_object.text_coords() = std::move(parser.ctx().default_text_coords);
  m_object.normals() = std::move(parser.ctx().default_normals);
}

}  // namespace mcpt::obj_parser
