#include "mcpt/common/geometry/aabb.hpp"

#include <limits>

#include <Eigen/Eigen>
#include <catch2/catch.hpp>

TEMPLATE_TEST_CASE("axis-aligned bounding box", "[geometry][aabb]", float, double) {

using AABB = mcpt::AABB<TestType>;
using Vector3 = Eigen::Matrix<TestType, 3, 1>;

Vector3 MIN_VERTEX{Vector3::Constant(std::numeric_limits<TestType>::lowest())};
Vector3 MAX_VERTEX{Vector3::Constant(std::numeric_limits<TestType>::max())};

SECTION("default ctor creates an invalid AABB") {
  AABB aabb;
  CHECK(aabb.min_vertex() == MAX_VERTEX);
  CHECK(aabb.max_vertex() == MIN_VERTEX);
  CHECK(aabb.center() == Vector3::Zero());
  CHECK(aabb.diagonal() == Vector3::Zero());
}

SECTION("other ctors") {
  AABB aabb(Vector3::Zero(), Vector3::Ones());
  CHECK(aabb.min_vertex() == Vector3::Zero());
  CHECK(aabb.max_vertex() == Vector3::Ones());
  CHECK(aabb.center() == Vector3::Constant(0.5));
  CHECK(aabb.diagonal() == Vector3::Ones());

  SECTION("copy ctor") {
    AABB aabb_copy = aabb;
    CHECK(aabb_copy.min_vertex() == aabb.min_vertex());
    CHECK(aabb_copy.max_vertex() == aabb.max_vertex());
    CHECK(aabb_copy.center() == aabb.center());
    CHECK(aabb_copy.diagonal() == aabb.diagonal());
  }
}

SECTION("update AABB") {
  AABB aabb;
  REQUIRE(aabb.min_vertex() == MAX_VERTEX);
  REQUIRE(aabb.max_vertex() == MIN_VERTEX);
  REQUIRE(aabb.center() == Vector3::Zero());
  REQUIRE(aabb.diagonal() == Vector3::Zero());

  AABB aabb_extended(Vector3::Constant(-2.0), Vector3::Constant(2.0));
  REQUIRE(aabb_extended.min_vertex() == Vector3::Constant(-2.0));
  REQUIRE(aabb_extended.max_vertex() == Vector3::Constant(2.0));
  REQUIRE(aabb_extended.center() == Vector3::Zero());
  REQUIRE(aabb_extended.diagonal() == Vector3::Constant(4.0));

  SECTION("update with vertex [1 1 1]") {
    aabb.Update(Vector3::Ones());
    SECTION("finish") {
      aabb.Finish();
      CHECK(aabb.min_vertex() == Vector3::Ones());
      CHECK(aabb.max_vertex() == Vector3::Ones());
      CHECK(aabb.center() == Vector3::Ones());
      CHECK(aabb.diagonal() == Vector3::Zero());
    }

    SECTION("update with vertex [-1 -1 -1]") {
      aabb.Update(-Vector3::Ones());
      SECTION("finish") {
        aabb.Finish();
        CHECK(aabb.min_vertex() == -Vector3::Ones());
        CHECK(aabb.max_vertex() == Vector3::Ones());
        CHECK(aabb.center() == Vector3::Zero());
        CHECK(aabb.diagonal() == Vector3::Constant(2.0));
      }

      SECTION("update with AABB([-2 -2 -2], [2 2 2])") {
        aabb.Update(aabb_extended);
        SECTION("finish") {
          aabb.Finish();
          CHECK(aabb.min_vertex() == Vector3::Constant(-2.0));
          CHECK(aabb.max_vertex() == Vector3::Constant(2.0));
          CHECK(aabb.center() == Vector3::Zero());
          CHECK(aabb.diagonal() == Vector3::Constant(4.0));
        }

        SECTION("update with invalid AABB") {
          aabb.Update(AABB{});
          aabb.Finish();
          CHECK(aabb.min_vertex() == Vector3::Constant(-2.0));
          CHECK(aabb.max_vertex() == Vector3::Constant(2.0));
          CHECK(aabb.center() == Vector3::Zero());
          CHECK(aabb.diagonal() == Vector3::Constant(4.0));
        }
      }
    }
  }
}

SECTION("test envelop") {
  AABB aabb(-Vector3::Ones(), Vector3::Ones());
  REQUIRE(aabb.min_vertex() == -Vector3::Ones());
  REQUIRE(aabb.max_vertex() == Vector3::Ones());
  REQUIRE(aabb.center() == Vector3::Zero());
  REQUIRE(aabb.diagonal() == Vector3::Constant(2.0));

  CHECK(aabb.Envelop(Vector3::Zero()));
  CHECK(aabb.Envelop(-Vector3::Ones()));
  CHECK(aabb.Envelop(Vector3::Ones()));

  CHECK_FALSE(aabb.Envelop(Vector3(0.0, 0.0, 2.0)));
  CHECK_FALSE(aabb.Envelop(Vector3(-2.0, 0.0, 0.0)));
}

}
