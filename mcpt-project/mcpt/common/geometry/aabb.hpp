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
      : m_min_vertex(min_v), m_max_vertex(max_v) {
    Finish();
  }

  auto& min_vertex() const noexcept { return m_min_vertex; }
  auto& max_vertex() const noexcept { return m_max_vertex; }
  auto& center() const noexcept { return m_center; };
  auto& diagonal() const noexcept { return m_diagonal; };

  template <typename U>
  void Update(const Eigen::MatrixBase<U>& v) {
    m_min_vertex = m_min_vertex.cwiseMin(v);
    m_max_vertex = m_max_vertex.cwiseMax(v);
  }

  void Update(const AABB& aabb) {
    m_min_vertex = m_min_vertex.cwiseMin(aabb.min_vertex());
    m_max_vertex = m_max_vertex.cwiseMax(aabb.max_vertex());
  }

  void Finish() {
    ASSERT((m_min_vertex.array() <= m_max_vertex.array()).all());
    m_center = (m_min_vertex + m_max_vertex) / 2;
    m_diagonal = m_max_vertex - m_min_vertex;
  }

  template <typename U>
  bool Envelop(const Eigen::MatrixBase<U>& v) const {
    return (v.array() >= m_min_vertex.array()).all() && (v.array() <= m_max_vertex.array()).all();
  }

private:
  using Vector3 = Eigen::Matrix<T, 3, 1>;

  Vector3 m_min_vertex{Vector3::Constant(std::numeric_limits<T>::max())};
  Vector3 m_max_vertex{Vector3::Constant(std::numeric_limits<T>::lowest())};
  Vector3 m_center{Vector3::Zero()};
  Vector3 m_diagonal{Vector3::Zero()};
};

}  // namespace mcpt
