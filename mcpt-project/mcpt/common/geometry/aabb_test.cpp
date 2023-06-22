#include "mcpt/common/geometry/aabb.hpp"

#include <limits>

#include <Eigen/Eigen>
#include <catch2/catch.hpp>

CATCH_TEMPLATE_TEST_CASE("axis-aligned bounding box", "[geometry][aabb]", float, double) {

using AABB = mcpt::AABB<TestType>;
using Vector3 = Eigen::Matrix<TestType, 3, 1>;

Vector3 MIN_VERTEX{Vector3::Constant(std::numeric_limits<TestType>::lowest())};
Vector3 MAX_VERTEX{Vector3::Constant(std::numeric_limits<TestType>::max())};

CATCH_SECTION("default ctor creates an invalid AABB") {
  AABB aabb;
  CATCH_CHECK(aabb.min_vertex() == MAX_VERTEX);
  CATCH_CHECK(aabb.max_vertex() == MIN_VERTEX);
  CATCH_CHECK(aabb.center() == Vector3::Zero());
  CATCH_CHECK(aabb.diagonal() == Vector3::Zero());
}

CATCH_SECTION("other ctors") {
  AABB aabb(Vector3::Zero(), Vector3::Ones());
  CATCH_CHECK(aabb.min_vertex() == Vector3::Zero());
  CATCH_CHECK(aabb.max_vertex() == Vector3::Ones());
  CATCH_CHECK(aabb.center() == Vector3::Constant(0.5));
  CATCH_CHECK(aabb.diagonal() == Vector3::Ones());

  CATCH_SECTION("copy ctor") {
    AABB aabb_copy = aabb;
    CATCH_CHECK(aabb_copy.min_vertex() == aabb.min_vertex());
    CATCH_CHECK(aabb_copy.max_vertex() == aabb.max_vertex());
    CATCH_CHECK(aabb_copy.center() == aabb.center());
    CATCH_CHECK(aabb_copy.diagonal() == aabb.diagonal());
  }
}

CATCH_SECTION("update AABB") {
  AABB aabb;
  CATCH_REQUIRE(aabb.min_vertex() == MAX_VERTEX);
  CATCH_REQUIRE(aabb.max_vertex() == MIN_VERTEX);
  CATCH_REQUIRE(aabb.center() == Vector3::Zero());
  CATCH_REQUIRE(aabb.diagonal() == Vector3::Zero());

  AABB aabb_extended(Vector3::Constant(-2.0), Vector3::Constant(2.0));
  CATCH_REQUIRE(aabb_extended.min_vertex() == Vector3::Constant(-2.0));
  CATCH_REQUIRE(aabb_extended.max_vertex() == Vector3::Constant(2.0));
  CATCH_REQUIRE(aabb_extended.center() == Vector3::Zero());
  CATCH_REQUIRE(aabb_extended.diagonal() == Vector3::Constant(4.0));

  CATCH_SECTION("update with vertex [1 1 1]") {
    aabb.Update(Vector3::Ones());
    CATCH_SECTION("finish") {
      aabb.Finish();
      CATCH_CHECK(aabb.min_vertex() == Vector3::Ones());
      CATCH_CHECK(aabb.max_vertex() == Vector3::Ones());
      CATCH_CHECK(aabb.center() == Vector3::Ones());
      CATCH_CHECK(aabb.diagonal() == Vector3::Zero());
    }

    CATCH_SECTION("update with vertex [-1 -1 -1]") {
      aabb.Update(-Vector3::Ones());
      CATCH_SECTION("finish") {
        aabb.Finish();
        CATCH_CHECK(aabb.min_vertex() == -Vector3::Ones());
        CATCH_CHECK(aabb.max_vertex() == Vector3::Ones());
        CATCH_CHECK(aabb.center() == Vector3::Zero());
        CATCH_CHECK(aabb.diagonal() == Vector3::Constant(2.0));
      }

      CATCH_SECTION("update with AABB([-2 -2 -2], [2 2 2])") {
        aabb.Update(aabb_extended);
        CATCH_SECTION("finish") {
          aabb.Finish();
          CATCH_CHECK(aabb.min_vertex() == Vector3::Constant(-2.0));
          CATCH_CHECK(aabb.max_vertex() == Vector3::Constant(2.0));
          CATCH_CHECK(aabb.center() == Vector3::Zero());
          CATCH_CHECK(aabb.diagonal() == Vector3::Constant(4.0));
        }

        CATCH_SECTION("update with invalid AABB") {
          aabb.Update(AABB{});
          aabb.Finish();
          CATCH_CHECK(aabb.min_vertex() == Vector3::Constant(-2.0));
          CATCH_CHECK(aabb.max_vertex() == Vector3::Constant(2.0));
          CATCH_CHECK(aabb.center() == Vector3::Zero());
          CATCH_CHECK(aabb.diagonal() == Vector3::Constant(4.0));
        }
      }
    }
  }
}

CATCH_SECTION("test envelop") {
  AABB aabb(-Vector3::Ones(), Vector3::Ones());
  CATCH_REQUIRE(aabb.min_vertex() == -Vector3::Ones());
  CATCH_REQUIRE(aabb.max_vertex() == Vector3::Ones());
  CATCH_REQUIRE(aabb.center() == Vector3::Zero());
  CATCH_REQUIRE(aabb.diagonal() == Vector3::Constant(2.0));

  CATCH_CHECK(aabb.Envelop(Vector3::Zero()));
  CATCH_CHECK(aabb.Envelop(-Vector3::Ones()));
  CATCH_CHECK(aabb.Envelop(Vector3::Ones()));

  CATCH_CHECK_FALSE(aabb.Envelop(Vector3(0.0, 0.0, 2.0)));
  CATCH_CHECK_FALSE(aabb.Envelop(Vector3(-2.0, 0.0, 0.0)));
}

}
