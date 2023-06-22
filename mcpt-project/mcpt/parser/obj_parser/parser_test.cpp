#include "mcpt/parser/obj_parser/parser.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string_view>
#include <unordered_map>
#include <utility>

#include <catch2/catch.hpp>

#include "mcpt/common/object/object.hpp"

#include "mcpt/parser/obj_parser/test_mock.hpp"

TEST_CASE("`.obj' parser parse a example correctly", "[parser][obj_parser]") {

static constexpr size_t NUM_DEFAULT_VERTICES = 228;
static constexpr size_t NUM_DEFAULT_TEXT_COORDS = 282;
static constexpr size_t NUM_DEFAULT_NORMALS = 256;

static const std::unordered_map MESH_GROUPS{std::make_pair("initialShadingGroup", 3U),
                                            std::make_pair("lambert3SG", 1U),
                                            std::make_pair("lambert2SG", 1U),
                                            std::make_pair("blinn2SG", 6U)};

auto mockfile = [](std::string_view filename, std::string_view filecontent) {
  auto path = std::filesystem::temp_directory_path() / filename;
  std::ofstream(path) << filecontent;
  return path;
};

SECTION("parse example `.obj' file without constructing BVH tree") {
  auto obj_path = mockfile(mcpt::obj_parser::MOCK_OBJ_FILENAME, mcpt::obj_parser::MOCK_OBJ_CONTENT);
  auto mtl_path = mockfile(mcpt::obj_parser::MOCK_MTL_FILENAME, mcpt::obj_parser::MOCK_MTL_CONTENT);
  INFO("mock `.obj' file generated at " << obj_path);
  INFO("mock `.mtl' file generated at " << mtl_path);

  mcpt::obj_parser::Parser parser(obj_path);
  const mcpt::Object& obj = parser.object();

  CHECK(obj.vertices().size() == NUM_DEFAULT_VERTICES);
  CHECK(obj.text_coords().size() == NUM_DEFAULT_TEXT_COORDS);
  CHECK(obj.normals().size() == NUM_DEFAULT_NORMALS);

  REQUIRE(obj.mesh_groups().size() == MESH_GROUPS.size());
  for (auto [material, num_faces] : MESH_GROUPS) {
    auto it = std::find_if(obj.mesh_groups().cbegin(), obj.mesh_groups().cend(), [&](auto& g) {
      return g.material == material;
    });
    bool found = (it != obj.mesh_groups().cend());

    CAPTURE(material, num_faces);
    REQUIRE(found);
    CHECK(it->material == material);
    CHECK(it->mesh_index.size() == num_faces);
  }
}

}
