#include "mcpt/parser/obj_parser/tokenizer.hpp"

#include <filesystem>

#include <Eigen/Eigen>
#include <catch2/catch.hpp>

#include "mcpt/parser/obj_parser/context.hpp"
#include "mcpt/parser/obj_parser/test_helper.hpp"

TEST_CASE("`.obj' tokenizer process one line correctly", "[parser][obj_parser][tokenizer]") {

std::filesystem::path mock_filepath = "/foo/bar.obj";
mcpt::obj_parser::Context ctx{mock_filepath};
mcpt::obj_parser::Tokenizer tokenizer(ctx);

SECTION("default context") {
  CHECK(ctx.filepath == mock_filepath);
  CHECK(ctx.linenum == 0);
  CHECK(ctx.materials.empty());
  CHECK(ctx.default_vertices.empty());
  CHECK(ctx.default_text_coords.empty());
  CHECK(ctx.default_normals.empty());
  CHECK(ctx.mesh_groups.empty());
  CHECK(ctx.associated_group == nullptr);
}

auto mock_ctx = ctx;

SECTION("parse `g default'") {
  tokenizer.Process("g default");
  REQUIRE_THAT(ctx, Equals(mock_ctx));

  SECTION("parse `v 0.0 0.0 0.0' `v 1.0 1.0 1.0'") {
    tokenizer.Process("v 0.0 0.0 0.0");
    tokenizer.Process("v 1.0 1.0 1.0");
    mock_ctx.default_vertices.push_back(Eigen::Vector3f::Zero());
    mock_ctx.default_vertices.push_back(Eigen::Vector3f::Ones());
    REQUIRE_THAT(ctx, Equals(mock_ctx));

    SECTION("parse `vt 1.0 1.0'") {
      tokenizer.Process("vt 1.0 1.0");
      mock_ctx.default_text_coords.push_back(Eigen::Vector2f::Ones());
      REQUIRE_THAT(ctx, Equals(mock_ctx));

      SECTION("parse `vn 1.0 1.0 1.0'") {
        tokenizer.Process("vn 1.0 1.0 1.0");
        mock_ctx.default_normals.push_back(Eigen::Vector3f::Ones());
        REQUIRE_THAT(ctx, Equals(mock_ctx));

        SECTION("parse one object group") {
          INFO("parse `g group'");
          tokenizer.Process("g group");
          INFO("parse `usemtl material'");
          tokenizer.Process("usemtl material");
          INFO("parse `f 1/1/1 2/1/1 2/1/1'");
          tokenizer.Process("f 1/1/1 2/1/1 2/1/1");

          mock_ctx.associated_group = &mock_ctx.mesh_groups["material"];
          mock_ctx.associated_group->material = "material";
          mock_ctx.associated_group->mesh_index.push_back({{0, 1, 1}, {0, 0, 0}, {0, 0, 0}});

          REQUIRE_THAT(ctx, Equals(mock_ctx));
        }
      }
    }
  }
}

SECTION("parse `s 1'") {
  tokenizer.Process("s 1");
  REQUIRE_THAT(ctx, Equals(mock_ctx));

  SECTION("parse `g group'") {
    tokenizer.Process("g group");
    REQUIRE_THAT(ctx, Equals(mock_ctx));

    SECTION("parse `s off'") {
      tokenizer.Process("s off");
      REQUIRE_THAT(ctx, Equals(mock_ctx));
    }
  }
}

}
