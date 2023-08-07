#pragma once

#include <cmath>
#include <algorithm>
#include <numeric>
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

  explicit Intersect(T precision) : m_precision(precision) { ASSERT(precision > 0.0); }

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
    if (std::abs(d_a - d_b) <= m_precision)
      return std::make_tuple(true, d_a, d_b);
    else
      return std::make_tuple(false, d_a, d_b);
  }

  template <typename U>
  bool Inside(const Eigen::MatrixBase<U>& p, const ConvexPolygon<T>& ply) const;

  // epsilon for line-plane parallelism test, should be non-negative
  T m_precision;
};

/**
 * AABB: three pairs of slabs        compute t_min/t_max for each pair of slabs
 *           |              |              |            |   /
 *      _____|______________|____      ____|____________|__/__
 *          /|             /|              |            | /t_max(Y)
 *       | / |          | / |              |            |/
 *   ____|/__|__________|/__|___           |       t_max/
 *       |   |          |   |              |       (X) /|
 *       |   |          |   |              |          / |
 *       |   | /        |   | /            |     dir /  |
 *       |   |/         |   |/             |        /   |
 *    ___|___|__________|___|______        |       /    |
 *       |  /|          |  /|              |      /     |
 *       | / |          | / |          ____|_____/______|_____
 *   ____|/_____________|/_______          |    /t_min  |
 *       /              /                  |   / (Y)    |
 *      /|             /|                  |  o         |
 *     / |            / |                  | / ray      |
 *                                         |/           |
 *   Z                                t_min/            |
 *   |    Y                           (X) /|            |
 *   |   /                               / |            |
 *   |  /
 *   | /                t_enter = max{t_min}, t_exit = min{t_max}
 *   |/________X        intersectant: t_enter <= t_exit and t_exit >= 0
 *   O
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
  if ((v_min.array() < v_max.array()).count() <= 1) {
    spdlog::error("invalid AABB: min = {}, max = {}", v_min.format(FMT), v_max.format(FMT));
    return false;
  }

  Vector3 slab_t_min = (v_min - r.point_a).cwiseProduct(r.direction_r);
  Vector3 slab_t_max = (v_max - r.point_a).cwiseProduct(r.direction_r);

  T t_en = std::numeric_limits<T>::lowest();
  T t_ex = std::numeric_limits<T>::max();
  for (Eigen::Index i = 0; i < 3; ++i) {
    if (r.direction.coeff(i) == 0.0) {
      // parallel or outside box
      if (r.point_a.coeff(i) < v_min.coeff(i) || r.point_a.coeff(i) > v_max.coeff(i))
        return false;
    } else {
      DASSERT(std::isfinite(slab_t_min.coeff(i)) && std::isfinite(slab_t_max.coeff(i)));
      auto [t_min, t_max] = std::minmax(slab_t_min.coeff(i), slab_t_max.coeff(i));
      t_en = std::max(t_en, t_min);
      t_ex = std::min(t_ex, t_max);
    }
  }
  return t_en <= t_ex && t_ex >= 0.0;
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
    Vector3 ab(ply.vertices[b] - ply.vertices[a]);
    Vector3 ax(p - ply.vertices[a]);
    cross.push_back(ab.cross(ax));
  }
  for (size_t a = 0; a < ply.vertices.size(); ++a) {
    size_t b = a + 1 < ply.vertices.size() ? a + 1 : 0;
    const auto& ab_x_ax = cross[a];
    const auto& bc_x_bx = cross[b];
    // at different sides
    if (ab_x_ax.dot(bc_x_bx) < 0.0)
      return false;
  }
  return true;
}

}  // namespace mcpt
