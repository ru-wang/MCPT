#include "mcpt/parser/obj_parser/line_parser.hpp"

#include <cctype>
#include <algorithm>
#include <iterator>

#include <spdlog/spdlog.h>

#include "mcpt/common/misc.hpp"

#include "mcpt/parser/obj_parser/tokenizer.hpp"

namespace mcpt::obj_parser {

void LineParser::Advance(std::istream& is) {
  Advance(SafelyGetLineString(is));
}

void LineParser::Advance(const std::string& statement) {
  // advance one line anyway
  spdlog::debug("parsing {} ({}): `{}'", ctx().filepath, ++ctx().linenum, statement);

  auto trim_start =
      std::find_if(statement.cbegin(), statement.cend(), [](char ch) { return !std::isspace(ch); });
  auto trimed = std::string_view(statement).substr(std::distance(statement.cbegin(), trim_start));

  // do nothing if it is an empty line or comment
  if (!trimed.empty() && trimed.front() != '#')
    Tokenizer(m_ctx).Process(trimed);
}

}  // namespace mcpt::obj_parser
