#include "mcpt/common/geometry/aabb.hpp"

#include <limits>

#include <Eigen/Eigen>
#include <catch2/catch_template_test_macros.hpp>

TEMPLATE_TEST_CASE("axis-aligned bounding box", "[geometry][aabb]", float, double) {

using AABB = mcpt::AABB<TestType>;
using Vector3 = Eigen::Matrix<TestType, 3, 1>;

Vector3 MIN_VERTEX{Vector3::Constant(std::numeric_limits<TestType>::lowest())};
Vector3 MAX_VERTEX{Vector3::Constant(std::numeric_limits<TestType>::max())};

SECTION("default ctor creates an invalid AABB") {
  AABB aabb;
  CHECK(aabb.min_vertex() == MAX_VERTEX);
  CHECK(aabb.max_vertex() == MIN_VERTEX);
}

SECTION("other ctors") {
  AABB aabb(Vector3::Zero(), Vector3::Ones());
  CHECK(aabb.min_vertex() == Vector3::Zero());
  CHECK(aabb.max_vertex() == Vector3::Ones());

  SECTION("copy ctor") {
    AABB aabb_copy = aabb;
    CHECK(aabb_copy.min_vertex() == aabb.min_vertex());
    CHECK(aabb_copy.max_vertex() == aabb.max_vertex());
  }
}

SECTION("update AABB") {
  AABB aabb;
  REQUIRE(aabb.min_vertex() == MAX_VERTEX);
  REQUIRE(aabb.max_vertex() == MIN_VERTEX);

  AABB aabb_extended(Vector3::Constant(-2.0), Vector3::Constant(2.0));
  REQUIRE(aabb_extended.min_vertex() == Vector3::Constant(-2.0));
  REQUIRE(aabb_extended.max_vertex() == Vector3::Constant(2.0));
  REQUIRE(aabb_extended.GetDiagonal() == Vector3::Constant(4.0));

  SECTION("update with vertex [1 1 1]") {
    aabb.Update(Vector3::Ones());
    SECTION("finish") {
      CHECK(aabb.min_vertex() == Vector3::Ones());
      CHECK(aabb.max_vertex() == Vector3::Ones());
      CHECK(aabb.GetDiagonal() == Vector3::Zero());
    }

    SECTION("update with vertex [-1 -1 -1]") {
      aabb.Update(-Vector3::Ones());
      SECTION("finish") {
        CHECK(aabb.min_vertex() == -Vector3::Ones());
        CHECK(aabb.max_vertex() == Vector3::Ones());
        CHECK(aabb.GetDiagonal() == Vector3::Constant(2.0));
      }

      SECTION("update with AABB([-2 -2 -2], [2 2 2])") {
        aabb.Update(aabb_extended);
        SECTION("finish") {
          CHECK(aabb.min_vertex() == Vector3::Constant(-2.0));
          CHECK(aabb.max_vertex() == Vector3::Constant(2.0));
          CHECK(aabb.GetDiagonal() == Vector3::Constant(4.0));
        }

        SECTION("update with invalid AABB") {
          aabb.Update(AABB{});
          CHECK(aabb.min_vertex() == Vector3::Constant(-2.0));
          CHECK(aabb.max_vertex() == Vector3::Constant(2.0));
          CHECK(aabb.GetDiagonal() == Vector3::Constant(4.0));
        }
      }
    }
  }
}

SECTION("test envelop") {
  AABB aabb(-Vector3::Ones(), Vector3::Ones());
  REQUIRE(aabb.min_vertex() == -Vector3::Ones());
  REQUIRE(aabb.max_vertex() == Vector3::Ones());
  REQUIRE(aabb.GetDiagonal() == Vector3::Constant(2.0));
}

}
