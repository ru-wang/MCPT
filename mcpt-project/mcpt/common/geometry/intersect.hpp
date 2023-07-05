#pragma once

#include <cmath>
#include <algorithm>
#include <tuple>
#include <vector>

#include <Eigen/Eigen>
#include <spdlog/spdlog.h>

#include "mcpt/common/geometry/aabb.hpp"
#include "mcpt/common/geometry/types.hpp"

namespace mcpt {

template <typename T>
class Intersect {
public:
  using Scalar = T;
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Vector4 = Eigen::Matrix<T, 4, 1>;

  explicit Intersect(T parallel_prec) : m_parallel_prec(parallel_prec) {}

  // return the homogeneous intersection of a line and a plane
  // return point at infinity if not intersectant
  Vector4 Get(const Line<T>& l, const Plane<T>& pi) const {
    auto [parallel, d_a, d_b] = TestParallel(l, pi);
    if (parallel)
      return Vector4::Zero();
    return ((l.point_a * d_b - l.point_b * d_a) / (d_b - d_a)).homogeneous();
  }

  // return the homogeneous intersection of a ray and a plane
  // return point at infinity if not intersectant
  Vector4 Get(const Ray<T>& r, const Plane<T>& pi) const {
    Vector4 x = Get(static_cast<const Line<T>&>(r), pi);
    if (x.w() == 0.0 || (x.template head<3>() - r.point_a).dot(r.direction) < 0.0)
      return Vector4::Zero();
    return x;
  }

  // return the homogeneous intersection of a ray and a convex polygon
  // return point at infinity if not intersectant
  Vector4 Get(const Line<T>& l, const ConvexPolygon<T>& ply) const {
    Vector4 x = Get(l, static_cast<const Plane<T>&>(ply));
    if (x.w() == 0.0 || !Inside(x.template head<3>(), ply))
      return Vector4::Zero();
    return x;
  }

  // return the homogeneous intersection of a ray and a convex polygon
  // return point at infinity if not intersectant
  Vector4 Get(const Ray<T>& r, const ConvexPolygon<T>& ply) const {
    Vector4 x = Get(r, static_cast<const Plane<T>&>(ply));
    if (x.w() == 0.0 || !Inside(x.template head<3>(), ply))
      return Vector4::Zero();
    return x;
  }

  // return whether a ray and an AABB intersect
  // TODO improve the performance
  bool Test(const Ray<T>& r, const AABB<T>& aabb) const;

private:
  std::tuple<bool, T, T> TestParallel(const Line<T>& l, const Plane<T>& pi) const {
    T d_a = l.point_a.homogeneous().dot(pi.coeffs);
    T d_b = l.point_b.homogeneous().dot(pi.coeffs);
    if (std::abs(d_a - d_b) <= m_parallel_prec)
      return std::make_tuple(true, d_a, d_b);
    else
      return std::make_tuple(false, d_a, d_b);
  }

  bool TestDegenerated(const Ray<T>& r, const AABB<T>& aabb) const;

  template <typename U>
  bool Inside(const Eigen::MatrixBase<U>& p, const ConvexPolygon<T>& ply) const;

  T m_parallel_prec;
};

/**
 *          b4______________b3(max)
 *           /|            /|
 *          / |           / |
 *       b1/__|__________/b2|
 *         |  |          |  |
 *         |  |          |  |
 *         |  |          |  |
 *  Z      |  |          |  |
 *  |      |  |__________|__|
 *  |   Y  | /a4         | /a3
 *  |  /   |/____________|/
 *  | /    a1(min)       a2
 *  |/_______ X
 */
template <typename T>
bool Intersect<T>::Test(const Ray<T>& r, const AABB<T>& aabb) const {
  static const Eigen::IOFormat FMT{Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " "};

  // avoid computing invalid ray or AABB
  if ((r.direction.array() == 0.0).all()) {
    spdlog::error("invalid ray: direction = {}", r.direction.format(FMT));
    return false;
  }

  const auto& v_min = aabb.min_vertex();
  const auto& v_max = aabb.max_vertex();
  size_t dim = (v_min.array() < v_max.array()).count();

  if (dim <= 1) {
    spdlog::error("invalid AABB: min = {}, max = {}", v_min.format(FMT), v_max.format(FMT));
    return false;
  } else if (dim == 2) {
    return TestDegenerated(r, aabb);
  }

  // start point inside AABB
  if (aabb.Envelop(r.point_a))
    return true;

  const auto& a1 = v_min;
  Vector3 a2(v_max.x(), v_min.y(), v_min.z());
  Vector3 a3(v_max.x(), v_max.y(), v_min.z());
  Vector3 a4(v_min.x(), v_max.y(), v_min.z());

  Vector3 b1(v_min.x(), v_min.y(), v_max.z());
  Vector3 b2(v_max.x(), v_min.y(), v_max.z());
  const auto& b3 = v_max;
  Vector3 b4(v_min.x(), v_max.y(), v_max.z());

  ConvexPolygon<T> ply_d(a1, a2, a3, a4);
  ConvexPolygon<T> ply_u(b1, b2, b3, b4);

  ConvexPolygon<T> ply_l(a1, b1, b4, a4);
  ConvexPolygon<T> ply_r(a2, b2, b3, a3);

  ConvexPolygon<T> ply_b(a1, a2, b2, b1);
  ConvexPolygon<T> ply_f(a4, b3, b3, b4);

  return Get(r, ply_u).w() != 0.0 || Get(r, ply_l).w() != 0.0 || Get(r, ply_f).w() != 0.0 ||
         Get(r, ply_d).w() != 0.0 || Get(r, ply_r).w() != 0.0 || Get(r, ply_b).w() != 0.0;
}

template <typename T>
bool Intersect<T>::TestDegenerated(const Ray<T>& r, const AABB<T>& aabb) const {
  const auto& v_min = aabb.min_vertex();
  const auto& v_max = aabb.max_vertex();

  Eigen::Index null_dim = 0;
  for (;; ++null_dim) {
    if (v_min[null_dim] == v_max[null_dim])
      break;
  }
  switch (null_dim) {
    case 0: {
      Vector3 a4(v_min.x(), v_max.y(), v_min.z());
      Vector3 b1(v_min.x(), v_min.y(), v_max.z());
      Vector3 b4(v_min.x(), v_max.y(), v_max.z());
      return Get(r, ConvexPolygon<T>(v_min, b1, b4, a4)).w() != 0.0;
    }
    case 1: {
      Vector3 a2(v_max.x(), v_min.y(), v_min.z());
      Vector3 b1(v_min.x(), v_min.y(), v_max.z());
      Vector3 b2(v_max.x(), v_min.y(), v_max.z());
      return Get(r, ConvexPolygon<T>(v_min, a2, b2, b1)).w() != 0.0;
    }
    case 2: {
      Vector3 a2(v_max.x(), v_min.y(), v_min.z());
      Vector3 a3(v_max.x(), v_max.y(), v_min.z());
      Vector3 a4(v_min.x(), v_max.y(), v_min.z());
      return Get(r, ConvexPolygon<T>(v_min, a2, a3, a4)).w() != 0.0;
    }
    default: ASSERT_FAIL(); return false;
  }
}

/**
 *               X
 *              /|\
 *             / | \
 *  E_________/__|__\___D
 *  |        /   |   \  |
 *  |       /    |    \ |
 *  |     Y/     |     \|
 *  |     /\     |     / C
 *  |    /  \    |    /
 *  |   /    \   |   /
 *  |  /      \  |  /
 *  | /        \ | /
 *  |/__________\|/
 *  A            B
 */
template <typename T>
template <typename U>
bool Intersect<T>::Inside(const Eigen::MatrixBase<U>& p, const ConvexPolygon<T>& ply) const {
  std::vector<Vector3> cross;
  for (size_t a = 0; a < ply.vertices.size(); ++a) {
    size_t b = a + 1 < ply.vertices.size() ? a + 1 : 0;
    Vector3 ab(ply.vertices.at(b) - ply.vertices.at(a));
    Vector3 ax(p - ply.vertices.at(a));
    cross.push_back(ab.cross(ax));
  }
  for (size_t a = 0; a < ply.vertices.size(); ++a) {
    size_t b = a + 1 < ply.vertices.size() ? a + 1 : 0;
    const auto& ab_x_ax = cross.at(a);
    const auto& bc_x_bx = cross.at(b);
    // at different sides
    if (ab_x_ax.dot(bc_x_bx) < 0.0)
      return false;
  }
  return true;
}

}  // namespace mcpt
