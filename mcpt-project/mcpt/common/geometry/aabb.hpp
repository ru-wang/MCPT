#pragma once

#include <Eigen/Eigen>

namespace mcpt {

template <typename T>
class AABB {
public:
  using Scalar = T;

  template <typename T0, typename T1>
  AABB(const Eigen::MatrixBase<T0>& min_v, const Eigen::MatrixBase<T1>& max_v) {
    Reset(min_v, max_v);
  }

  auto& min_vertex() const noexcept { return m_min_vertex; }
  auto& max_vertex() const noexcept { return m_max_vertex; }
  auto& center() const noexcept { return m_center; };
  auto& diagonal() const noexcept { return m_diagonal; };

  template <typename T0, typename T1>
  void Reset(const Eigen::MatrixBase<T0>& min_v, const Eigen::MatrixBase<T1>& max_v) {
    m_min_vertex = min_v;
    m_max_vertex = max_v;

    m_center = (m_min_vertex + m_max_vertex) / 2;
    m_diagonal = m_max_vertex - m_min_vertex;
  }

  template <typename U>
  void UpdateMin(const Eigen::MatrixBase<U>& v) {
    m_min_vertex = (m_min_vertex.array() < v.array()).select(m_min_vertex, v);
  }

  template <typename U>
  void UpdateMax(const Eigen::MatrixBase<U>& v) {
    m_max_vertex = (m_max_vertex.array() > v.array()).select(m_max_vertex, v);
  }

  void UpdateCenterDiag() {
    m_center = (m_min_vertex + m_max_vertex) / 2;
    m_diagonal = m_max_vertex - m_min_vertex;
  }

  template <typename U>
  bool Envelop(const Eigen::MatrixBase<U>& v) const {
    return (v.template head<3>().array() >= m_min_vertex.template head<3>().array()).all() &&
           (v.template head<3>().array() <= m_max_vertex.template head<3>().array()).all();
  }

private:
  Eigen::Matrix<T, 3, 1> m_min_vertex;
  Eigen::Matrix<T, 3, 1> m_max_vertex;

  Eigen::Matrix<T, 3, 1> m_center;
  Eigen::Matrix<T, 3, 1> m_diagonal;
};

}  // namespace mcpt
