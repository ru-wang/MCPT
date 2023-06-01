#pragma once

#include <cmath>
#include <algorithm>
#include <vector>

#include <Eigen/Eigen>

#include "mcpt/common/geometry/aabb.hpp"
#include "mcpt/common/geometry/types.hpp"

namespace mcpt {

template <typename T>
class Intersect {
public:
  using Scalar = T;
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Vector4 = Eigen::Matrix<T, 4, 1>;

  explicit Intersect(T prec) : m_prec(prec) {}

  // return the homogeneous intersection of a line and a plane
  // return point at infinity if not intersectant
  Vector4 Get(const Line<T>& l, const Plane<T>& pi) const {
    T d_b = l.point_b.homogeneous().dot(pi.coeffs);
    T d_a = l.point_a.homogeneous().dot(pi.coeffs);
    if (approx(d_b, d_a))  // parallel to the plane
      return Vector4::Zero();
    else
      return ((l.point_a * d_b - l.point_b * d_a) / (d_b - d_a)).homogeneous();
  }

  // return the homogeneous intersection of a ray and a plane
  // return point at infinity if not intersectant
  Vector4 Get(const Ray<T>& r, const Plane<T>& pi) const {
    using Vector4 = Vector4;
    Vector4 x = Get(static_cast<const Line<T>&>(r), pi);
    if (x.w() == 0 || (x.template head<3>() - r.point_a).dot(r.direction) < 0)
      return Vector4::Zero();
    else
      return x;
  }

  // return the homogeneous intersection of a ray and a convex polygon
  // return point at infinity if not intersectant
  Vector4 Get(const Ray<T>& r, const ConvexPolygon<T>& ply) const {
    Vector4 x = Get(r, static_cast<const Plane<T>&>(ply));
    if (x.w() == 0)
      return Vector4::Zero();

    std::vector<Vector3> cross;
    for (size_t a = 0; a < ply.vertices.size(); ++a) {
      size_t b = a + 1 < ply.vertices.size() ? a + 1 : 0;
      Vector3 ab(ply.vertices.at(b) - ply.vertices.at(a));
      Vector3 ax(x.template head<3>() - ply.vertices.at(a));
      cross.push_back(ab.cross(ax));
    }

    for (size_t i = 0; i < ply.vertices.size(); ++i) {
      // on the edge
      if (approx(cross.at(i).norm(), 0))
        return Vector4::Zero();
      // at different sides
      if (i + 1 < ply.vertices.size()) {
        if (cross.at(i).dot(cross.at(i + 1)) <= 0)
          return Vector4::Zero();
      }
    }

    return x;
  }

  // return whether a ray and an AABB intersect
  // TODO improve the performance
  bool Test(const Ray<T>& r, const AABB<T>& aabb) const {
    using Vector3 = Vector3;

    // start point inside AABB
    if (aabb.Envelop(r.point_a))
      return true;

    const auto& v_min = aabb.min_vertex();
    const auto& v_max = aabb.max_vertex();

    //         b4______________b3(max)
    //          /|            /|
    //         / |           / |
    //      b1/__|__________/b2|
    //        |  |          |  |
    //        |  |          |  |
    //        |  |          |  |
    // Z      |  |          |  |
    // |      |  |__________|__|
    // |   Y  | /a4         | /a3
    // |  /   |/____________|/
    // | /    a1(min)       a2
    // |/_______ X

    Vector3 a1 = v_min;
    Vector3 a2(v_max.x(), v_min.y(), v_min.z());
    Vector3 a3(v_max.x(), v_max.y(), v_min.z());
    Vector3 a4(v_min.x(), v_max.y(), v_min.z());

    Vector3 b1(v_min.x(), v_min.y(), v_max.z());
    Vector3 b2(v_max.x(), v_min.y(), v_max.z());
    Vector3 b3 = v_max;
    Vector3 b4(v_min.x(), v_max.y(), v_max.z());

    ConvexPolygon<float> ply_u(b1, b2, b3, b4);
    ConvexPolygon<float> ply_d(a1, a2, a3, a4);

    ConvexPolygon<float> ply_l(a1, b1, b4, a4);
    ConvexPolygon<float> ply_r(a2, b2, b3, a3);

    ConvexPolygon<float> ply_f(a4, b3, b3, b4);
    ConvexPolygon<float> ply_b(a1, a2, b2, b1);

    return Get(r, ply_u).w() != 0 || Get(r, ply_l).w() != 0 || Get(r, ply_f).w() != 0 ||
           Get(r, ply_d).w() != 0 || Get(r, ply_r).w() != 0 || Get(r, ply_b).w() != 0;
  }

private:
  bool approx(T lhs, T rhs) const {
    return std::abs(lhs - rhs) <= m_prec * std::min(std::abs(lhs), std::abs(rhs));
  }

  T m_prec;
};

}  // namespace mcpt
