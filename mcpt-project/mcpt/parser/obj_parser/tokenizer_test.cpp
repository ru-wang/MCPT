#include "mcpt/parser/obj_parser/tokenizer.hpp"

#include <filesystem>

#include <Eigen/Eigen>
#include <catch2/catch.hpp>

#include "mcpt/parser/obj_parser/context.hpp"
#include "mcpt/parser/obj_parser/test_helper.hpp"

CATCH_TEST_CASE("`.obj' tokenizer process one line correctly", "[parser][obj_parser][tokenizer]") {

std::filesystem::path mock_filepath = "/foo/bar.obj";
mcpt::obj_parser::Context ctx{mock_filepath};
mcpt::obj_parser::Tokenizer tokenizer(ctx);

CATCH_SECTION("default context") {
  CATCH_CHECK(ctx.filepath == mock_filepath);
  CATCH_CHECK(ctx.linenum == 0);
  CATCH_CHECK(ctx.materials.empty());
  CATCH_CHECK(ctx.default_vertices.empty());
  CATCH_CHECK(ctx.default_text_coords.empty());
  CATCH_CHECK(ctx.default_normals.empty());
  CATCH_CHECK(ctx.mesh_groups.empty());
  CATCH_CHECK(ctx.smooth_group.empty());
  CATCH_CHECK(ctx.associated_group == nullptr);
}

auto mock_ctx = ctx;

CATCH_SECTION("parse `g default'") {
  tokenizer.Process("g default");
  CATCH_REQUIRE_THAT(ctx, Equals(mock_ctx));

  CATCH_SECTION("parse `v 0.0 0.0 0.0' `v 1.0 1.0 1.0'") {
    tokenizer.Process("v 0.0 0.0 0.0");
    tokenizer.Process("v 1.0 1.0 1.0");
    mock_ctx.default_vertices.push_back(Eigen::Vector3f::Zero());
    mock_ctx.default_vertices.push_back(Eigen::Vector3f::Ones());
    CATCH_REQUIRE_THAT(ctx, Equals(mock_ctx));

    CATCH_SECTION("parse `vt 1.0 1.0'") {
      tokenizer.Process("vt 1.0 1.0");
      mock_ctx.default_text_coords.push_back(Eigen::Vector2f::Ones());
      CATCH_REQUIRE_THAT(ctx, Equals(mock_ctx));

      CATCH_SECTION("parse `vn 1.0 1.0 1.0'") {
        tokenizer.Process("vn 1.0 1.0 1.0");
        mock_ctx.default_normals.push_back(Eigen::Vector3f::Ones());
        CATCH_REQUIRE_THAT(ctx, Equals(mock_ctx));

        CATCH_SECTION("parse one object group") {
          CATCH_INFO("parse `g group'");
          tokenizer.Process("g group");
          CATCH_INFO("parse `usemtl material'");
          tokenizer.Process("usemtl material");
          CATCH_INFO("parse `f 1/1/1 2/1/1 2/1/1'");
          tokenizer.Process("f 1/1/1 2/1/1 2/1/1");

          mock_ctx.associated_group = &mock_ctx.mesh_groups["group"];
          mock_ctx.associated_group->material = "material";
          mock_ctx.associated_group->mesh_index.push_back({{0, 1, 1}, {0, 0, 0}, {0, 0, 0}});

          CATCH_REQUIRE_THAT(ctx, Equals(mock_ctx));
        }
      }
    }
  }
}

CATCH_SECTION("parse `s 1'") {
  tokenizer.Process("s 1");
  mock_ctx.smooth_group = "1";
  CATCH_REQUIRE_THAT(ctx, Equals(mock_ctx));

  CATCH_SECTION("parse `g group'") {
    tokenizer.Process("g group");
    mock_ctx.associated_group = &mock_ctx.mesh_groups["group"];
    mock_ctx.associated_group->smooth_group = mock_ctx.smooth_group;
    CATCH_REQUIRE_THAT(ctx, Equals(mock_ctx));

    CATCH_SECTION("parse `s off'") {
      tokenizer.Process("s off");
      mock_ctx.smooth_group = "off";
      CATCH_REQUIRE_THAT(ctx, Equals(mock_ctx));
    }
  }
}

}
