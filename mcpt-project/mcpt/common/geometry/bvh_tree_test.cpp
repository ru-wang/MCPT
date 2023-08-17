#include "mcpt/common/geometry/bvh_tree.hpp"

#include <deque>
#include <filesystem>
#include <fstream>
#include <string_view>

#include <catch2/catch_test_macros.hpp>

#include "mcpt/common/object/object.hpp"
#include "mcpt/parser/obj_parser/parser.hpp"
#include "mcpt/parser/obj_parser/test_mock.hpp"

TEST_CASE("bvh_tree", "[geometry][bvh_tree]") {

using BVHTree = mcpt::BVHTree<float>;

auto mockfile = [](std::string_view filename, std::string_view filecontent) {
  auto path = std::filesystem::temp_directory_path() / filename;
  std::ofstream(path) << filecontent;
  return path;
};

auto obj_path = mockfile(mcpt::obj_parser::MOCK_OBJ_FILENAME, mcpt::obj_parser::MOCK_OBJ_CONTENT);
auto mtl_path = mockfile(mcpt::obj_parser::MOCK_MTL_FILENAME, mcpt::obj_parser::MOCK_MTL_CONTENT);
INFO("mock `.obj' file generated at " << obj_path);
INFO("mock `.mtl' file generated at " << mtl_path);

mcpt::obj_parser::Parser parser(obj_path);
mcpt::Object& object = parser.object();

SECTION("traverse the BVH tree") {
  // count number of meshes
  size_t num_meshes = 0;
  for (const auto& groups : object.mesh_groups())
    num_meshes += groups.mesh_index.size();
  CAPTURE(num_meshes);

  BVHTree bvh_tree = object.CreateBVHTree();

  // count number of leaves
  size_t num_bvh_leaves = 0;
  for (std::deque queue{bvh_tree.root.get()}; !queue.empty(); queue.pop_front()) {
    auto node = queue.front();
    if (node->l_child)
      queue.push_back(node->l_child.get());
    if (node->r_child)
      queue.push_back(node->r_child.get());
    num_bvh_leaves += node->mesh.has_value();

    CHECK((node->aabb.min_vertex().array() <= node->aabb.max_vertex().array()).all());
    CHECK((node->aabb.min_vertex().array() < node->aabb.max_vertex().array()).count() > 1);

    // no orphan child
    CHECK(node->mesh.has_value() == (!node->l_child && !node->r_child));

    if (node->l_child && node->r_child) {
      const auto& l = node->l_child;
      const auto& r = node->r_child;
      CHECK(node->aabb.min_vertex() == l->aabb.min_vertex().cwiseMin(r->aabb.min_vertex()));
      CHECK(node->aabb.max_vertex() == l->aabb.max_vertex().cwiseMax(r->aabb.max_vertex()));
    }
  }

  CHECK(num_meshes == num_bvh_leaves);
}

}
