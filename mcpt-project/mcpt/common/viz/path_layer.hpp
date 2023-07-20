#pragma once

#include <array>
#include <mutex>
#include <vector>

#include <Eigen/Eigen>
#include <cheers/layer/layer.hpp>

#include "mcpt/renderer/path_tracer.hpp"

namespace mcpt {

class PathLayer : public cheers::Layer {
public:
  ~PathLayer() noexcept override = default;

  void OnCreateRenderer() override;
  void OnDestroyRenderer() override;

  void OnUpdateImFrame() override;
  void OnUpdateRenderData() override;
  void OnRenderLayer(const float* matrix_vp) override;

  void AddPaths(const Eigen::Vector3f& start, const std::vector<PathTracer::ReversePath>& rpaths) {
    static constexpr float NORMAL_LENGTH = 0.1F;

    std::lock_guard lock(render_data_mutex());
    auto& normal_data = m_render_data.normal_data.emplace_back();
    auto& path_data = m_render_data.path_data.emplace_back();

    Eigen::Vector3f last_point = start;
    for (const auto& rpath : rpaths) {
      normal_data.insert(normal_data.cend(), rpath.point.begin(), rpath.point.end());
      normal_data.push_back(rpath.point.x() + rpath.normal.x() * NORMAL_LENGTH);
      normal_data.push_back(rpath.point.y() + rpath.normal.y() * NORMAL_LENGTH);
      normal_data.push_back(rpath.point.z() + rpath.normal.z() * NORMAL_LENGTH);

      path_data.insert(path_data.cend(), last_point.begin(), last_point.end());
      path_data.insert(path_data.cend(), rpath.point.begin(), rpath.point.end());

      last_point = rpath.point;
    }

    m_render_data.dirty = true;
  }

private:
  struct RenderData {
    bool dirty = false;
    std::vector<std::vector<float>> normal_data;
    std::vector<std::vector<float>> path_data;

    // ImGui data
    bool fluoroscopy = false;
    bool draw_normal = true;
    float line_width = 1.0F;
    std::array<float, 4> normal_color{0.22F, 0.78F, 0.78F, 0.22F};
    std::array<float, 4> path_color{0.78F, 0.22F, 0.78F, 0.22F};
  };

  RenderData m_render_data;
};

}  // namespace mcpt
