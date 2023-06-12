#pragma once

#include <functional>
#include <istream>
#include <string>

#include "mcpt/parser/obj_parser/context.hpp"

namespace mcpt::obj_parser {

class LineParser {
public:
  explicit LineParser(std::reference_wrapper<Context> ctx) : m_ctx(ctx) {}

  void Advance(std::istream& is);
  void Advance(const std::string& statement);

private:
  auto& ctx() noexcept { return m_ctx.get(); }

  std::reference_wrapper<Context> m_ctx;
};

}  // namespace mcpt::obj_parser
