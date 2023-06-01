#include "mcpt/parser/obj_line_parser.hpp"

#include <filesystem>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Eigen>
#include <catch2/catch.hpp>
#include <spdlog/fmt/fmt.h>

#include "mcpt/common/object/material.hpp"
#include "mcpt/common/object/mesh.hpp"
#include "mcpt/common/object/object.hpp"

template <int Rows>
struct fmt::formatter<std::vector<Eigen::Matrix<float, Rows, 1>>> {
  using Type = std::vector<Eigen::Matrix<float, Rows, 1>>;
  inline static const Eigen::IOFormat FORMAT{
      Eigen::StreamPrecision, Eigen::DontAlignCols, "", ",", "", "", "(", ")"};
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    // check if reached the end of the range
    auto it = ctx.begin();
    auto end = ctx.end();
    if (it != end && *it != '}')
      throw fmt::format_error("invalid format");
    // return an iterator past the end of the parsed range
    return it;
  }
  template <typename FormatContext>
  auto format(const Type& v, FormatContext& ctx) const -> decltype(ctx.out()) {
    for (auto it = v.cbegin(); it != v.cend(); ++it) {
      if (it + 1 == v.cend())
        fmt::format_to(ctx.out(), "{}", it->format(FORMAT));
      else
        fmt::format_to(ctx.out(), "{}, ", it->format(FORMAT));
    }
    return ctx.out();
  }
};

template <typename Value>
struct fmt::formatter<std::pair<const std::string, Value>> {
  using Type = std::pair<const std::string, Value>;
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    // check if reached the end of the range
    auto it = ctx.begin();
    auto end = ctx.end();
    if (it != end && *it != '}')
      throw format_error("invalid format");
    // return an iterator past the end of the parsed range
    return it;
  }
  template <typename FormatContext>
  auto format(const Type& v, FormatContext& ctx) const -> decltype(ctx.out()) {
    return fmt::format_to(ctx.out(), "'{}':{}", v.first, v.second);
  }
};

template <>
struct fmt::formatter<mcpt::Material> {
  using Type = mcpt::Material;
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    // check if reached the end of the range
    auto it = ctx.begin();
    auto end = ctx.end();
    if (it != end && *it != '}')
      throw format_error("invalid format");
    // return an iterator past the end of the parsed range
    return it;
  }
  template <typename FormatContext>
  auto format(const Type& v, FormatContext& ctx) const -> decltype(ctx.out()) {
    return fmt::format_to(
        ctx.out(),
        "{{illum={}, Kd={}, Ka={}, Ks={}, Ns={}, Ni={}, Tr={}}}",
        v.illum, v.Kd, v.Ka, v.Ks, v.Ns, v.Ni, v.Tr);
  }
};

template <>
struct fmt::formatter<mcpt::MeshIndex> {
  using Type = mcpt::MeshIndex;
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    // check if reached the end of the range
    auto it = ctx.begin();
    auto end = ctx.end();
    if (it != end && *it != '}')
      throw format_error("invalid format");
    // return an iterator past the end of the parsed range
    return it;
  }
  template <typename FormatContext>
  auto format(const Type& v, FormatContext& ctx) const -> decltype(ctx.out()) {
    return fmt::format_to(ctx.out(),
                          "[{},{},{}]",
                          fmt::join(v.vindex, "/"),
                          fmt::join(v.tindex, "/"),
                          fmt::join(v.nindex, "/"));
  }
};

template <>
struct fmt::formatter<mcpt::MeshIndexGroup> {
  using Type = mcpt::MeshIndexGroup;
  constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
    // check if reached the end of the range
    auto it = ctx.begin();
    auto end = ctx.end();
    if (it != end && *it != '}')
      throw format_error("invalid format");
    // return an iterator past the end of the parsed range
    return it;
  }
  template <typename FormatContext>
  auto format(const Type& v, FormatContext& ctx) const -> decltype(ctx.out()) {
    return fmt::format_to(
        ctx.out(),
        "{{smooth={}, material='{}', mesh_index=[{}]}}",
        v.smooth, v.material, fmt::join(v.mesh_index, ", "));
  }
};

using Context = mcpt::ObjParser::LineParser::Context;

template <>
struct Catch::StringMaker<Context> {
  static std::string convert(const Context& ctx) {
    return fmt::format(
        "{{\n"
        "  filepath: {}\n"
        "  linenum: {}\n"
        "  materials: {{{}}}\n"
        "  default_vertices: [{}]\n"
        "  default_text_coords: [{}]\n"
        "  default_normals: [{}]\n"
        "  mesh_groups: {{{}}}\n"
        "  smooth_switch: {}\n"
        "  bound_group: {}\n"
        "}}\n",
        ctx.filepath,
        ctx.linenum,
        fmt::join(ctx.materials, ", "),
        ctx.default_vertices,
        ctx.default_text_coords,
        ctx.default_normals,
        fmt::join(ctx.mesh_groups, ", "),
        ctx.smooth_switch,
        fmt::ptr(ctx.bound_group));
  }
};

struct Equals : public Catch::MatcherBase<Context> {
  std::reference_wrapper<const Context> rhs;

  Equals(std::reference_wrapper<const Context> ctx) : rhs(ctx) {}
  ~Equals() override = default;

  bool match(const Context& lhs) const override {
    return lhs.filepath == rhs.get().filepath &&
           lhs.linenum == rhs.get().linenum &&
           lhs.materials == rhs.get().materials &&
           lhs.default_vertices == rhs.get().default_vertices &&
           lhs.default_text_coords == rhs.get().default_text_coords &&
           lhs.default_normals == rhs.get().default_normals &&
           lhs.mesh_groups == rhs.get().mesh_groups &&
           lhs.smooth_switch == rhs.get().smooth_switch &&
           ( (lhs.bound_group == nullptr && rhs.get().bound_group == nullptr) ||
             *lhs.bound_group == *rhs.get().bound_group );
  }

  std::string describe() const override {
    return "EQUALS TO\n" + Catch::StringMaker<Context>::convert(rhs.get());
  }
};

CATCH_TEST_CASE("`.obj' line parser parse one line correctly", "[utility][obj_line_parser]") {

std::filesystem::path mock_filepath = "/foo/bar.obj";
mcpt::ObjParser::LineParser parser{{mock_filepath}};

CATCH_SECTION("default parser context") {
  CATCH_CHECK(parser.ctx().filepath == mock_filepath);
  CATCH_CHECK(parser.ctx().linenum == 0);
  CATCH_CHECK(parser.ctx().materials.empty());
  CATCH_CHECK(parser.ctx().default_vertices.empty());
  CATCH_CHECK(parser.ctx().default_text_coords.empty());
  CATCH_CHECK(parser.ctx().default_normals.empty());
  CATCH_CHECK(parser.ctx().mesh_groups.empty());
  CATCH_CHECK(parser.ctx().smooth_switch == 0);
  CATCH_CHECK(parser.ctx().bound_group == nullptr);
}

auto mock_ctx = parser.ctx();

CATCH_SECTION("parse `g default'") {
  parser.Advance("g default");
  ++mock_ctx.linenum;
  CATCH_REQUIRE_THAT(parser.ctx(), Equals(mock_ctx));

  CATCH_SECTION("parse `v 0.0 0.0 0.0' `v 1.0 1.0 1.0'") {
    parser.Advance("v 0.0 0.0 0.0");
    parser.Advance("v 1.0 1.0 1.0");
    mock_ctx.default_vertices.push_back(Eigen::Vector3f::UnitW());
    mock_ctx.default_vertices.push_back(Eigen::Vector3f::Ones());
    mock_ctx.linenum += 2;
    CATCH_REQUIRE_THAT(parser.ctx(), Equals(mock_ctx));

    CATCH_SECTION("parse `vt 1.0 1.0'") {
      parser.Advance("vt 1.0 1.0");
      mock_ctx.default_text_coords.push_back(Eigen::Vector2f::Ones());
      ++mock_ctx.linenum;
      CATCH_REQUIRE_THAT(parser.ctx(), Equals(mock_ctx));

      CATCH_SECTION("parse `vn 1.0 1.0 1.0'") {
        parser.Advance("vn 1.0 1.0 1.0");
        mock_ctx.default_normals.push_back(Eigen::Vector3f::Ones());
        ++mock_ctx.linenum;
        CATCH_REQUIRE_THAT(parser.ctx(), Equals(mock_ctx));

        CATCH_SECTION("parse one object group") {
          CATCH_INFO("parse `g group'");
          parser.Advance("g group");
          CATCH_INFO("parse `usemtl material'");
          parser.Advance("usemtl material");
          CATCH_INFO("parse `f 1/1/1 2/1/1 2/1/1'");
          parser.Advance("f 1/1/1 2/1/1 2/1/1");

          mock_ctx.bound_group = &mock_ctx.mesh_groups["group"];
          mock_ctx.bound_group->material = "material";
          mock_ctx.bound_group->mesh_index.push_back({{0, 1, 1}, {0, 0, 0}, {0, 0, 0}});
          mock_ctx.linenum += 3;

          CATCH_REQUIRE_THAT(parser.ctx(), Equals(mock_ctx));
        }
      }
    }
  }
}

CATCH_SECTION("parse `s 1'") {
  parser.Advance("s 1");
  mock_ctx.smooth_switch = 1;
  ++mock_ctx.linenum;
  CATCH_REQUIRE_THAT(parser.ctx(), Equals(mock_ctx));

  CATCH_SECTION("parse `  #comment'") {
    parser.Advance("  #comment");
    ++mock_ctx.linenum;
    CATCH_REQUIRE_THAT(parser.ctx(), Equals(mock_ctx));

    CATCH_SECTION("parse `g group'") {
      parser.Advance("g group");
      mock_ctx.bound_group = &mock_ctx.mesh_groups["group"];
      mock_ctx.bound_group->smooth = 1;
      ++mock_ctx.linenum;
      CATCH_REQUIRE_THAT(parser.ctx(), Equals(mock_ctx));

      CATCH_SECTION("parse `s off'") {
        parser.Advance("s off");
        mock_ctx.smooth_switch = 0;
        ++mock_ctx.linenum;
        CATCH_REQUIRE_THAT(parser.ctx(), Equals(mock_ctx));
      }
    }
  }
}

}
