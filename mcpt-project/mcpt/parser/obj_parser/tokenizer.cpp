#include "mcpt/parser/obj_parser/tokenizer.hpp"

#include <charconv>
#include <regex>
#include <string>

#include "mcpt/common/assert.hpp"
#include "mcpt/parser/mtl_parser/parser.hpp"

#define ASSERT_PARSE(assertion, msg_tpl, ...) \
  ASSERT(assertion, "fail to parse {} ({}): " msg_tpl, ctx().filepath, ctx().linenum, ##__VA_ARGS__)
#define ASSERT_PARSE_FAIL(msg_tpl, ...) \
  ASSERT_FAIL("fail to parse {} ({}): " msg_tpl, ctx().filepath, ctx().linenum, ##__VA_ARGS__)

namespace mcpt::obj_parser {

namespace {

const std::regex SPLIT_STATEMENT{R"((\S+)\s+(\S+.*))"};
const std::regex SPLIT_SPACE{R"(\s+)"};
const std::regex SPLIT_SLASH{R"((\d+)/(\d+)/(\d+))"};

template <typename T>
bool as_number(const std::sub_match<const char*>& match, T& number) {
  auto ret = std::from_chars(match.first, match.second, number);
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
  if (identifier == "mtllib") {
    // declaration: material file relative path
    ProcMTLFilename(declaration);
  } else if (identifier == "g") {
    // declaration: group name
    ProcGroupName(declaration);
  } else if (identifier == "v") {
    // declaration: x y z coordinates
    auto& v = ctx().default_vertices.emplace_back();
    ProcNumericVector<3>(declaration, v.data());
  } else if (identifier == "vt") {
    // declaration: u v coordinates
    auto& vt = ctx().default_text_coords.emplace_back();
    ProcNumericVector<2>(declaration, vt.data());
  } else if (identifier == "vn") {
    // declaration: x y z coordinates
    auto& vn = ctx().default_normals.emplace_back();
    ProcNumericVector<3>(declaration, vn.data());
  } else if (identifier == "f") {
    // declaration: index tokens v/vt/vn
    ProcMeshIndex(declaration);
  } else if (identifier == "s") {
    // declaration: off or smooth value
    // ignore
  } else if (identifier == "usemtl") {
    // declaration: material name
    ProcMaterialDecl(declaration);
  } else {
    ASSERT_PARSE_FAIL("unknown identifier `{}'", identifier);
  }
}

void Tokenizer::ProcMTLFilename(std::string_view mtl_filename) {
  auto mtl_filepath = std::filesystem::absolute(ctx().filepath).parent_path() / mtl_filename;
  mtl_parser::Parser mtl_parser(mtl_filepath);
  for (auto&& [mtl_name, mtl] : mtl_parser.materials()) {
    bool inserted = ctx().materials.emplace(mtl_name, std::move(mtl)).second;
    ASSERT_PARSE(inserted, "material `{}' in `{}' already exists", mtl_name, mtl_filepath);
  }
}

void Tokenizer::ProcGroupName(std::string_view group_name) {
  ctx().associated_group = nullptr;
}

template <size_t Size, typename T>
void Tokenizer::ProcNumericVector(std::string_view tokens, T* vector) {
  std::cregex_token_iterator token_first(tokens.cbegin(), tokens.cend(), SPLIT_SPACE, -1);
  std::cregex_token_iterator token_last;
  ASSERT_PARSE(std::distance(token_first, token_last) == Size, "must provide {} tokens", Size);

  for (size_t i = 0; i < Size; ++i, ++token_first)
    ASSERT_PARSE(as_number(*token_first, vector[i]), "invalid token `{}'", token_first->str());
}

void Tokenizer::ProcMeshIndex(std::string_view tokens) {
  ASSERT_PARSE(ctx().associated_group, "must be associated to group other than `default'");
  auto& mesh_index = ctx().associated_group->mesh_index.emplace_back();

  std::cregex_token_iterator token_first(tokens.cbegin(), tokens.cend(), SPLIT_SPACE, -1);
  std::cregex_token_iterator token_last;
  ASSERT_PARSE(std::distance(token_first, token_last) >= 3, "must provide at least 3 index tokens");

  while (token_first != token_last) {
    std::cmatch matches;
    bool matched = std::regex_match(token_first->first, token_first->second, matches, SPLIT_SLASH);
    ASSERT_PARSE(matched, "index token must be in the form of `v/vt/vn'");

    int vindex;
    int tindex;
    int nindex;
    ASSERT_PARSE(as_number(matches[1], vindex), "invalid token `{}'", matches[1].str());
    ASSERT_PARSE(as_number(matches[2], tindex), "invalid token `{}'", matches[2].str());
    ASSERT_PARSE(as_number(matches[3], nindex), "invalid token `{}'", matches[3].str());
    ASSERT_PARSE(vindex > 0 && tindex > 0 && nindex > 0, "index token must be positive");

    mesh_index.vindex.push_back(vindex - 1);
    mesh_index.tindex.push_back(tindex - 1);
    mesh_index.nindex.push_back(nindex - 1);

    ++token_first;
  }
}

void Tokenizer::ProcMaterialDecl(std::string_view material_name) {
  ctx().associated_group = &ctx().mesh_groups[std::string(material_name)];
  ctx().associated_group->material = material_name;
}

}  // namespace mcpt::obj_parser

#undef ASSERT_PARSE
#undef ASSERT_PARSE_FAIL
