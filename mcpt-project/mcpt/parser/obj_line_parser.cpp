#include "mcpt/parser/obj_line_parser.hpp"

#include <cctype>
#include <algorithm>
#include <iterator>
#include <regex>
#include <sstream>
#include <utility>

#include "mcpt/common/assert.hpp"

#include "mcpt/parser/misc.hpp"
#include "mcpt/parser/mtl_parser.hpp"

#define ASSERT_PARSE(assertion, msg_tpl, ...) \
  ASSERT(assertion, "fail to parse {} ({}): " msg_tpl, m_ctx.filepath, m_ctx.linenum, ##__VA_ARGS__)
#define ASSERT_PARSE_FAIL(msg_tpl, ...) \
  ASSERT_FAIL("fail to parse {} ({}): " msg_tpl, m_ctx.filepath, m_ctx.linenum, ##__VA_ARGS__)

namespace mcpt {

namespace {

const std::regex RE_STATEMENT{R"((\S+)\s+(\S+.*))"};
const std::regex RE_TOKEN{R"(\s+)"};
const std::regex RE_SPLIT{R"((\d+)/(\d+)/(\d+))"};

}  // namespace

ObjParser::LineParser::LineParser(Context init_ctx) : m_ctx(std::move(init_ctx)) {}

void ObjParser::LineParser::Advance(std::istream& is) {
  Advance(SafelyGetLineString(is));
}

void ObjParser::LineParser::Advance(const std::string& statement) {
  spdlog::debug("parsing {}: `{}'", m_ctx.filepath, statement);

  std::string trimed;
  auto first_non_space =
      std::find_if(statement.cbegin(), statement.cend(), [](char ch) { return !std::isspace(ch); });
  if (first_non_space != statement.cend())
    trimed = statement.substr(std::distance(statement.cbegin(), first_non_space));

  if (!trimed.empty() && trimed.front() != '#') {
    std::smatch id_decl;
    bool matched = std::regex_match(trimed, id_decl, RE_STATEMENT);
    ASSERT_PARSE(matched, "invalid statement `{}'", statement);
    Advance(id_decl[1], id_decl[2]);
  }
  ++m_ctx.linenum;
}

void ObjParser::LineParser::Advance(const std::string& identifier, const std::string& declaration) {
  if (identifier == "mtllib") {
    // declaration: material file relative path
    ParseMTLFilename(declaration);
  } else if (identifier == "g") {
    // declaration: group name
    ParseGroupName(declaration);
  } else if (identifier == "v") {
    // declaration: x y z coordinates
    auto& v = m_ctx.default_vertices.emplace_back();
    ParseNumericVector<3>(declaration, v);
  } else if (identifier == "vt") {
    // declaration: u v coordinates
    auto& vt = m_ctx.default_text_coords.emplace_back();
    ParseNumericVector<2>(declaration, vt);
  } else if (identifier == "vn") {
    // declaration: x y z coordinates
    auto& vn = m_ctx.default_normals.emplace_back();
    ParseNumericVector<3>(declaration, vn);
  } else if (identifier == "f") {
    // declaration: index tokens v/vt/vn
    ParseMeshIndex(declaration);
  } else if (identifier == "s") {
    // declaration: off or smooth value
    ParseSmoothSwitch(declaration);
  } else if (identifier == "usemtl") {
    // declaration: material name
    ParseMaterialDecl(declaration);
  } else {
    ASSERT_PARSE_FAIL("unknown identifier `{}'", identifier);
  }
}

void ObjParser::LineParser::ParseMTLFilename(const std::filesystem::path& mtl_filename) {
  auto mtl_filepath = std::filesystem::absolute(m_ctx.filepath).parent_path() / mtl_filename;
  MtlParser mtl_parser(mtl_filepath);
  for (auto&& [mtl_name, mtl] : mtl_parser.materials()) {
    bool inserted = m_ctx.materials.emplace(mtl_name, std::move(mtl)).second;
    ASSERT(inserted, "material `{}' in `{}' already exists", mtl_name, mtl_filepath);
  }
}

void ObjParser::LineParser::ParseGroupName(const std::string& group_name) {
  if (group_name == "default") {
    // following attributes (v/vt/vn) will added to default groups
    m_ctx.bound_group = nullptr;
  } else {
    // following attributes (s/f) will added to default groups
    m_ctx.bound_group = &m_ctx.mesh_groups.try_emplace(group_name).first->second;
    m_ctx.bound_group->smooth = m_ctx.smooth_switch;
  }
}

template <Eigen::Index Size, typename Derived>
void ObjParser::LineParser::ParseNumericVector(const std::string& tokens,
                                               Eigen::MatrixBase<Derived>& vector) {
  std::sregex_token_iterator token_first(tokens.cbegin(), tokens.cend(), RE_TOKEN, -1);
  std::sregex_token_iterator token_last;
  ASSERT_PARSE(std::distance(token_first, token_last) == Size, "must provide {} tokens", Size);

  for (Eigen::Index i = 0; i < Size; ++i, ++token_first)
    vector[i] = std::stof(token_first->str());
}

void ObjParser::LineParser::ParseMeshIndex(const std::string& tokens) {
  ASSERT_PARSE(m_ctx.bound_group, "must be bound to group other than `default'");
  auto& mesh_index = m_ctx.bound_group->mesh_index.emplace_back();

  std::sregex_token_iterator token_first(tokens.cbegin(), tokens.cend(), RE_TOKEN, -1);
  std::sregex_token_iterator token_last;
  ASSERT_PARSE(std::distance(token_first, token_last) >= 3, "must provide at least 3 index tokens");

  while (token_first != token_last) {
    std::smatch matches;
    bool matched = std::regex_match(token_first->first, token_first->second, matches, RE_SPLIT);
    ASSERT_PARSE(matched, "index token must be in the form of `v/vt/vn'");

    int vindex = std::stoi(matches[1]);
    int tindex = std::stoi(matches[2]);
    int nindex = std::stoi(matches[3]);
    ASSERT_PARSE(vindex > 0 && tindex > 0 && nindex > 0, "index token must be positive");

    mesh_index.vindex.push_back(vindex - 1);
    mesh_index.tindex.push_back(tindex - 1);
    mesh_index.nindex.push_back(nindex - 1);

    ++token_first;
  }
}

void ObjParser::LineParser::ParseSmoothSwitch(const std::string& smooth_decl) {
  m_ctx.smooth_switch = (smooth_decl == "off") ? 0 : std::stoi(smooth_decl);
}

void ObjParser::LineParser::ParseMaterialDecl(const std::string& material_name) {
  ASSERT_PARSE(m_ctx.bound_group, "must be bound to group other than `default'");
  m_ctx.bound_group->material = material_name;
}

}  // namespace mcpt

#undef ASSERT_PARSE
#undef ASSERT_PARSE_FAIL
