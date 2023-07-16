#pragma once

#include <array>
#include <mutex>
#include <vector>

#include <Eigen/Eigen>
#include <cheers/layer/layer.hpp>

#include "mcpt/common/geometry/types.hpp"

namespace mcpt {

class ShapeLayer : public cheers::Layer {
public:
  ~ShapeLayer() noexcept override = default;

  void OnCreateRenderer() override;
  void OnDestroyRenderer() override;

  void OnUpdateImFrame() override;
  void OnUpdateRenderData() override;
  void OnRenderLayer(const float* matrix_vp) override;

  template <typename InputIt>
  void AddPoints(InputIt first, InputIt last) {
    std::lock_guard lock(render_data_mutex());
    auto& point_data = m_render_data.point_data.emplace_back();
    while (first != last) {
      const auto& point = *first;
      point_data.insert(point_data.cend(), point.begin(), point.end());
      ++first;
    }
    m_render_data.dirty = true;
  }

  template <typename InputIt>
  void AddLines(InputIt first, InputIt last) {
    std::lock_guard lock(render_data_mutex());
    auto& line_data = m_render_data.line_data.emplace_back();
    while (first != last) {
      const auto& line = *first;
      line_data.insert(line_data.cend(), line.point_a.begin(), line.point_a.end());
      line_data.insert(line_data.cend(), line.point_b.begin(), line.point_b.end());
      ++first;
    }
    m_render_data.dirty = true;
  }

  void AddPoints(const std::vector<Eigen::Vector3f>& points) {
    AddPoints(points.cbegin(), points.cend());
  }

  void AddLines(const std::vector<Line<float>>& lines) { AddLines(lines.cbegin(), lines.cend()); }

private:
  struct RenderData {
    bool dirty = false;
    std::vector<std::vector<float>> point_data;
    std::vector<std::vector<float>> line_data;

    // ImGui data
    bool draw_point = true;
    bool draw_line = true;
    float point_size = 1.0F;
    float line_width = 1.0F;
    std::array<float, 4> point_color{0.78F, 0.22F, 0.78F, 1.0F};
    std::array<float, 4> line_color{0.22F, 0.78F, 0.78F, 1.0F};
  };

  RenderData m_render_data;
};

}  // namespace mcpt

