#include "mcpt/parser/obj_parser/test_helper.hpp"

using Context = mcpt::obj_parser::Context;

using Material = mcpt::Material;
using MeshIndex = mcpt::MeshIndex;
using MeshIndexGroup = mcpt::MeshIndexGroup;

auto fmt::formatter<Material>::parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
  // check if reached the end of the range
  auto it = ctx.begin();
  auto end = ctx.end();
  if (it != end && *it != '}')
    throw format_error("invalid format");
  // return an iterator past the end of the parsed range
  return it;
}

auto fmt::formatter<MeshIndex>::parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
  // check if reached the end of the range
  auto it = ctx.begin();
  auto end = ctx.end();
  if (it != end && *it != '}')
    throw format_error("invalid format");
  // return an iterator past the end of the parsed range
  return it;
}

auto fmt::formatter<MeshIndexGroup>::parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
  // check if reached the end of the range
  auto it = ctx.begin();
  auto end = ctx.end();
  if (it != end && *it != '}')
    throw format_error("invalid format");
  // return an iterator past the end of the parsed range
  return it;
}

std::string Catch::StringMaker<Context>::convert(const Context& ctx) {
  return fmt::format(
      "{{\n"
      "  filepath: {}\n"
      "  linenum: {}\n"
      "  materials: {{{}}}\n"
      "  default_vertices: [{}]\n"
      "  default_text_coords: [{}]\n"
      "  default_normals: [{}]\n"
      "  mesh_groups: {{{}}}\n"
      "  associated_group: {}\n"
      "}}\n",
      ctx.filepath,
      ctx.linenum,
      fmt::join(ctx.materials, ", "),
      ctx.default_vertices,
      ctx.default_text_coords,
      ctx.default_normals,
      fmt::join(ctx.mesh_groups, ", "),
      fmt::ptr(ctx.associated_group));
}

bool Equals::match(const Context& lhs) const {
  return lhs.filepath == rhs.get().filepath &&
         lhs.linenum == rhs.get().linenum &&
         lhs.materials == rhs.get().materials &&
         lhs.default_vertices == rhs.get().default_vertices &&
         lhs.default_text_coords == rhs.get().default_text_coords &&
         lhs.default_normals == rhs.get().default_normals &&
         lhs.mesh_groups == rhs.get().mesh_groups &&
         ( (lhs.associated_group == nullptr && rhs.get().associated_group == nullptr) ||
           *lhs.associated_group == *rhs.get().associated_group );
}

std::string Equals::describe() const {
  return "EQUALS TO\n" + Catch::StringMaker<Context>::convert(rhs.get());
}
