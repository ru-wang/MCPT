#include "mcpt/parser/mtl_parser/tokenizer.hpp"

#include <charconv>
#include <regex>
#include <string>

#include "mcpt/common/assert.hpp"

#define ASSERT_PARSE(assertion, msg_tpl, ...) \
  ASSERT(assertion, "fail to parse {} ({}): " msg_tpl, ctx().filepath, ctx().linenum, ##__VA_ARGS__)
#define ASSERT_PARSE_FAIL(msg_tpl, ...) \
  ASSERT_FAIL("fail to parse {} ({}): " msg_tpl, ctx().filepath, ctx().linenum, ##__VA_ARGS__)

namespace mcpt::mtl_parser {

namespace {

const std::regex SPLIT_STATEMENT{R"(([\S^#]+)\s+([^#]+).*)"};
const std::regex SPLIT_SPACE{R"(\s+)"};

template <typename T>
bool as_number(const std::sub_match<const char*>& match, T& number) {
  auto ret = std::from_chars(match.first, match.second, number);
  return ret.ec == std::errc{};
}

template <typename T>
bool as_number(const std::string_view& token, T& number) {
  auto ret = std::from_chars(token.cbegin(), token.cend(), number);
  return ret.ec == std::errc{};
}

}  // namespace

void Tokenizer::Process(std::string_view statement) {
  std::cmatch id_decl;
  bool matched = std::regex_match(statement.cbegin(), statement.cend(), id_decl, SPLIT_STATEMENT);
  ASSERT_PARSE(matched, "invalid statement `{}'", statement);
  Proc(std::string_view(id_decl[1].first, id_decl[1].length()),
       std::string_view(id_decl[2].first, id_decl[2].length()));
}

void Tokenizer::Proc(std::string_view identifier, std::string_view declaration) {
  if (identifier == "newmtl") {
    ctx().associated_material = &ctx().materials[std::string(declaration)];
    return;
  }
  ASSERT_PARSE(ctx().associated_material, "must be associated to a meterial name");

  if (identifier == "illum") {
    ProcNumber(declaration, ctx().associated_material->illum);
    ASSERT_PARSE(ctx().associated_material->illum == 4, "only `4' allowed for illumination type");
  } else if (identifier == "Ke") {
    auto& Ke = ctx().associated_material->Ke;
    ProcNumericVector<3>(declaration, Ke.data());
    ASSERT_PARSE((Ke.array() >= 0.0F).all(), "`Ke' out of range");
  } else if (identifier == "Kd") {
    auto& Kd = ctx().associated_material->Kd;
    ProcNumericVector<3>(declaration, Kd.data());
    ASSERT_PARSE((Kd.array() >= 0.0F).all() && (Kd.array() <= 1.0F).all(), "`Kd' out of range");
  } else if (identifier == "Ka") {
    auto& Ka = ctx().associated_material->Ka;
    ProcNumericVector<3>(declaration, Ka.data());
    ASSERT_PARSE((Ka.array() >= 0.0F).all() && (Ka.array() <= 1.0F).all(), "`Ka' out of range");
  } else if (identifier == "Ks") {
    auto& Ks = ctx().associated_material->Ks;
    ProcNumericVector<3>(declaration, Ks.data());
    ASSERT_PARSE((Ks.array() >= 0.0F).all() && (Ks.array() <= 1.0F).all(), "`Ks' out of range");
  } else if (identifier == "Ns") {
    auto& Ns = ctx().associated_material->Ns;
    ProcNumber(declaration, Ns);
    ASSERT_PARSE(Ns >= 0.0F && Ns <= 1000.0F, "`Ns' out of range");
  } else if (identifier == "Ni") {
    auto& Ni = ctx().associated_material->Ni;
    ProcNumber(declaration, Ni);
    ASSERT_PARSE(Ni >= 1.0F && Ni <= 10.0F, "`Ni' out of range");
  } else if (identifier == "Tr") {
    auto& Tr = ctx().associated_material->Tr;
    ProcNumber(declaration, Tr);
    ASSERT_PARSE(Tr >= 0.0F && Tr <= 1.0F, "`Tr' out of range");
  } else {
    ASSERT_PARSE_FAIL("unknown identifier `{}'", identifier);
  }
}

template <typename T>
void Tokenizer::ProcNumber(std::string_view token, T& number) {
  ASSERT_PARSE(as_number(token, number), "invalid token `{}'", token);
}

template <size_t Size, typename T>
void Tokenizer::ProcNumericVector(std::string_view tokens, T* vector) {
  std::cregex_token_iterator token_first(tokens.cbegin(), tokens.cend(), SPLIT_SPACE, -1);
  std::cregex_token_iterator token_last;
  ASSERT_PARSE(std::distance(token_first, token_last) == Size, "must provide {} tokens", Size);

  for (size_t i = 0; i < Size; ++i, ++token_first)
    ASSERT_PARSE(as_number(*token_first, vector[i]), "invalid token `{}'", token_first->str());
}

}  // namespace mcpt::mtl_parser
