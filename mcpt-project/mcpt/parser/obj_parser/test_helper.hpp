#pragma once

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

#include "mcpt/parser/obj_parser/context.hpp"

template <int Rows>
struct fmt::formatter<std::vector<Eigen::Matrix<float, Rows, 1>>> {
  using Type = std::vector<Eigen::Matrix<float, Rows, 1>>;

  inline static const Eigen::IOFormat FORMAT{
      Eigen::StreamPrecision, Eigen::DontAlignCols, "", ",", "", "", "(", ")"};

  auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
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

  auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
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

  auto parse(format_parse_context& ctx) -> decltype(ctx.begin());

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

  auto parse(format_parse_context& ctx) -> decltype(ctx.begin());

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

  auto parse(format_parse_context& ctx) -> decltype(ctx.begin());

  template <typename FormatContext>
  auto format(const Type& v, FormatContext& ctx) const -> decltype(ctx.out()) {
    return fmt::format_to(
        ctx.out(), "{{material='{}', mesh_index=[{}]}}", v.material, fmt::join(v.mesh_index, ", "));
  }
};

template <>
struct Catch::StringMaker<mcpt::obj_parser::Context> {
  static std::string convert(const mcpt::obj_parser::Context& ctx);
};

struct Equals : public Catch::MatcherBase<mcpt::obj_parser::Context> {
  std::reference_wrapper<const mcpt::obj_parser::Context> rhs;

  Equals(std::reference_wrapper<const mcpt::obj_parser::Context> ctx) : rhs(ctx) {}
  ~Equals() override = default;

  bool match(const mcpt::obj_parser::Context& lhs) const override;
  std::string describe() const override;
};
