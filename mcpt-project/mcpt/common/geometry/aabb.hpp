#pragma once

#include <limits>

#include <Eigen/Eigen>

#include "mcpt/common/assert.hpp"

namespace mcpt {

template <typename T>
class AABB {
public:
  using Scalar = T;

  AABB() = default;

  template <typename T0, typename T1>
  AABB(const Eigen::MatrixBase<T0>& min_v, const Eigen::MatrixBase<T1>& max_v)
      : m_min_vertex(min_v), m_max_vertex(max_v) {}

  auto& min_vertex() const noexcept { return m_min_vertex; }
  auto& max_vertex() const noexcept { return m_max_vertex; }

  template <typename U>
  void Update(const Eigen::MatrixBase<U>& v) {
    m_min_vertex = m_min_vertex.cwiseMin(v);
    m_max_vertex = m_max_vertex.cwiseMax(v);
  }

  void Update(const AABB& aabb) {
    m_min_vertex = m_min_vertex.cwiseMin(aabb.min_vertex());
    m_max_vertex = m_max_vertex.cwiseMax(aabb.max_vertex());
  }

  Eigen::Matrix<T, 3, 1> GetDiagonal() const { return m_max_vertex - m_min_vertex; };

private:
  using Vector3 = Eigen::Matrix<T, 3, 1>;

  Vector3 m_min_vertex{Vector3::Constant(std::numeric_limits<T>::max())};
  Vector3 m_max_vertex{Vector3::Constant(std::numeric_limits<T>::lowest())};
};

}  // namespace mcpt
