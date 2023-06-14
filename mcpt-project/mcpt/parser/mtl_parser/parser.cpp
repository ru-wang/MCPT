#include "mcpt/parser/mtl_parser/parser.hpp"

#include <cctype>
#include <algorithm>
#include <fstream>
#include <iterator>
#include <string_view>
#include <utility>

#include <spdlog/spdlog.h>

#include "mcpt/common/assert.hpp"
#include "mcpt/parser/misc.hpp"

#include "mcpt/parser/mtl_parser/tokenizer.hpp"

namespace mcpt::mtl_parser {

Parser::Parser(const std::filesystem::path& filepath) {
  Context ctx{filepath};

  std::ifstream ifs(filepath);
  ASSERT(ifs.is_open(), "failed to open {}", filepath);

  while (ifs && !ifs.eof())
    Advance(SafelyGetLineString(ifs), ctx);

  for (auto& [mtl_name, mtl] : ctx.materials) {
    auto inserted = m_materials.emplace(std::move(mtl_name), std::move(mtl)).second;
    ASSERT(inserted, "duplicate material name `{}' in {}", mtl_name, filepath);
  }
}

void Parser::Advance(const std::string& statement, Context& ctx) {
  // advance one line anyway
  spdlog::debug("parsing {} ({}): `{}'", ctx.filepath, ++ctx.linenum, statement);

  auto trim_start =
      std::find_if(statement.cbegin(), statement.cend(), [](char ch) { return !std::isspace(ch); });
  auto trimed = std::string_view(statement).substr(std::distance(statement.cbegin(), trim_start));

  // do nothing if it is an empty line or comment
  if (!trimed.empty() && trimed.front() != '#')
    Tokenizer(ctx).Process(trimed);
}

}  // namespace mcpt::mtl_parser
