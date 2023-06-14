#include "mcpt/common/geometry/intersect.hpp"

#include <Eigen/Eigen>
#include <catch2/catch.hpp>

CATCH_TEST_CASE("intersect compute intersection of line/ray and shape", "[geometry][intersect]") {

using Line = mcpt::Line<double>;
using Ray = mcpt::Ray<double>;
using Plane = mcpt::Plane<double>;
using Polygon = mcpt::ConvexPolygon<double>;

static constexpr double PRECISION = 1.0e-10;
mcpt::Intersect<double> intersect(PRECISION);

CATCH_SECTION("intersection of line and plane") {
  Eigen::Vector3d point_x = Eigen::Vector3d::UnitX();
  Eigen::Vector3d point_y = Eigen::Vector3d::UnitY();
  Eigen::Vector3d point_z = Eigen::Vector3d::UnitZ();

  Plane pi(point_x, point_y, point_z);
  CATCH_CAPTURE(pi.coeffs.transpose());

  CATCH_SECTION("on the plane") {
    Line l(point_x, point_y);
    Eigen::Vector4d p = intersect.Get(l, pi);
    CATCH_CHECK(p == Eigen::Vector4d::Zero());
  }

  CATCH_SECTION("parallel") {
    Eigen::Vector3d point_c = (point_x + point_y + point_z) / 3.0;
    CATCH_REQUIRE(point_c.homogeneous().dot(pi.coeffs) == Approx(0.0).margin(PRECISION));

    Line l(Eigen::Vector3d::Zero(), point_x - point_c);
    Eigen::Vector4d p = intersect.Get(l, pi);
    CATCH_CHECK(p == Eigen::Vector4d::Zero());
  }

  CATCH_SECTION("intersectant") {
    Line l(Eigen::Vector3d::Zero(), point_x);
    Eigen::Vector3d p = intersect.Get(l, pi).hnormalized();
    CATCH_CHECK(p.isApprox(point_x, PRECISION));
  }
}

CATCH_SECTION("intersection of line and triangular") {
  Eigen::Vector3d point_x = Eigen::Vector3d::UnitX();
  Eigen::Vector3d point_y = Eigen::Vector3d::UnitY();
  Eigen::Vector3d point_z = Eigen::Vector3d::UnitZ();

  Polygon tri(point_x, point_y, point_z);
  CATCH_CAPTURE(tri.coeffs.transpose());

  CATCH_SECTION("on the plane") {
    Line l(point_x, point_y);
    Eigen::Vector4d p = intersect.Get(l, tri);
    CATCH_CHECK(p == Eigen::Vector4d::Zero());
  }

  CATCH_SECTION("parallel") {
    Eigen::Vector3d point_c = (point_x + point_y + point_z) / 3.0;
    CATCH_REQUIRE(point_c.homogeneous().dot(tri.coeffs) == Approx(0.0).margin(PRECISION));

    Line l(Eigen::Vector3d::Zero(), point_x - point_c);
    Eigen::Vector4d p = intersect.Get(l, tri);
    CATCH_CHECK(p == Eigen::Vector4d::Zero());
  }

  CATCH_SECTION("intersection inside") {
    Eigen::Vector3d point_c = (point_x + point_y + point_z) / 3.0;
    CATCH_REQUIRE(point_c.homogeneous().dot(tri.coeffs) == Approx(0.0).margin(PRECISION));

    Line l(Eigen::Vector3d::Zero(), point_c);
    Eigen::Vector3d p = intersect.Get(l, tri).hnormalized();
    CATCH_CHECK(p.isApprox(point_c, PRECISION));
  }

  CATCH_SECTION("intersection outside") {
    Eigen::Vector3d point_c = point_x + point_x - point_y;
    CATCH_REQUIRE(point_c.homogeneous().dot(tri.coeffs) == Approx(0.0).margin(PRECISION));

    Line l(Eigen::Vector3d::Zero(), point_c);
    Eigen::Vector4d p = intersect.Get(l, tri);
    CATCH_CHECK(p == Eigen::Vector4d::Zero());
  }
}

CATCH_SECTION("intersection of ray and plane") {
  Eigen::Vector3d point_x = Eigen::Vector3d::UnitX();
  Eigen::Vector3d point_y = Eigen::Vector3d::UnitY();
  Eigen::Vector3d point_z = Eigen::Vector3d::UnitZ();

  Plane pi(point_x, point_y, point_z);
  CATCH_CAPTURE(pi.coeffs.transpose());

  CATCH_SECTION("on the plane") {
    Ray r(point_x, point_y - point_x);
    Eigen::Vector4d p = intersect.Get(r, pi);
    CATCH_CHECK(p == Eigen::Vector4d::Zero());
  }

  CATCH_SECTION("parallel") {
    Eigen::Vector3d point_c = (point_x + point_y + point_z) / 3.0;
    CATCH_REQUIRE(point_c.homogeneous().dot(pi.coeffs) == Approx(0.0).margin(PRECISION));

    Ray r(Eigen::Vector3d::Zero(), point_x - point_c);
    Eigen::Vector4d p = intersect.Get(r, pi);
    CATCH_CHECK(p == Eigen::Vector4d::Zero());
  }

  CATCH_SECTION("intersectant") {
    Ray r(Eigen::Vector3d::Zero(), point_x);
    Eigen::Vector3d p = intersect.Get(r, pi).hnormalized();
    CATCH_CHECK(p.isApprox(point_x, PRECISION));
  }

  CATCH_SECTION("no intersection") {
    Ray r(Eigen::Vector3d::Zero(), -point_x);
    Eigen::Vector4d p = intersect.Get(r, pi);
    CATCH_CHECK(p == Eigen::Vector4d::Zero());
  }
}

CATCH_SECTION("intersection of ray and triangular") {
  Eigen::Vector3d point_x = Eigen::Vector3d::UnitX();
  Eigen::Vector3d point_y = Eigen::Vector3d::UnitY();
  Eigen::Vector3d point_z = Eigen::Vector3d::UnitZ();

  Polygon tri(point_x, point_y, point_z);
  CATCH_CAPTURE(tri.coeffs.transpose());

  CATCH_SECTION("on the plane") {
    Ray r(point_x, point_y - point_x);
    Eigen::Vector4d p = intersect.Get(r, tri);
    CATCH_CHECK(p == Eigen::Vector4d::Zero());
  }

  CATCH_SECTION("parallel") {
    Eigen::Vector3d point_c = (point_x + point_y + point_z) / 3.0;
    CATCH_REQUIRE(point_c.homogeneous().dot(tri.coeffs) == Approx(0.0).margin(PRECISION));

    Ray r(Eigen::Vector3d::Zero(), point_x - point_c);
    Eigen::Vector4d p = intersect.Get(r, tri);
    CATCH_CHECK(p == Eigen::Vector4d::Zero());
  }

  CATCH_SECTION("intersection inside") {
    Eigen::Vector3d point_c = (point_x + point_y + point_z) / 3.0;
    CATCH_REQUIRE(point_c.homogeneous().dot(tri.coeffs) == Approx(0.0).margin(PRECISION));

    Ray r(Eigen::Vector3d::Zero(), point_c);
    Eigen::Vector3d p = intersect.Get(r, tri).hnormalized();
    CATCH_CHECK(p.isApprox(point_c, PRECISION));
  }

  CATCH_SECTION("intersection outside") {
    Eigen::Vector3d point_c = point_x + point_x - point_y;
    CATCH_REQUIRE(point_c.homogeneous().dot(tri.coeffs) == Approx(0.0).margin(PRECISION));

    Ray r(Eigen::Vector3d::Zero(), point_c);
    Eigen::Vector4d p = intersect.Get(r, tri);
    CATCH_CHECK(p == Eigen::Vector4d::Zero());
  }

  CATCH_SECTION("no intersection") {
    Ray r(Eigen::Vector3d::Zero(), -point_x);
    Eigen::Vector4d p = intersect.Get(r, tri);
    CATCH_CHECK(p == Eigen::Vector4d::Zero());
  }
}

}
