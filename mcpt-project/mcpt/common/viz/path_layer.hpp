#pragma once

#include <array>
#include <mutex>
#include <vector>

#include <Eigen/Eigen>
#include <cheers/layer/layer.hpp>

#include "mcpt/renderer/monte_carlo.hpp"

namespace mcpt {

class PathLayer : public cheers::Layer {
public:
  explicit PathLayer(unsigned int image_width) : m_image_width(image_width) {}
  ~PathLayer() noexcept override = default;

  void OnCreateRenderer() override;
  void OnDestroyRenderer() override;

  void OnUpdateImFrame() override;
  void OnUpdateRenderData() override;
  void OnRenderLayer(const float* matrix_vp) override;

  void AddPaths(const Eigen::Vector3f& start, const std::vector<MonteCarlo::RPath>& rpaths) {
    static constexpr float NORMAL_LENGTH = 0.2F;

    std::lock_guard lock(render_data_mutex());
    auto& normal_data = m_render_data.normal_data.emplace_back();
    auto& rpath_data = m_render_data.rpath_data.emplace_back();
    auto& lpath_data = m_render_data.lpath_data.emplace_back();

    Eigen::Vector3f last_point = start;
    for (const auto& [rpath, lpath] : rpaths) {
      normal_data.insert(normal_data.end(), rpath.point.begin(), rpath.point.end());
      normal_data.push_back(rpath.point.x() + rpath.normal.x() * NORMAL_LENGTH);
      normal_data.push_back(rpath.point.y() + rpath.normal.y() * NORMAL_LENGTH);
      normal_data.push_back(rpath.point.z() + rpath.normal.z() * NORMAL_LENGTH);

      rpath_data.insert(rpath_data.end(), last_point.begin(), last_point.end());
      rpath_data.insert(rpath_data.end(), rpath.point.begin(), rpath.point.end());

      if (lpath.has_value()) {
        lpath_data.insert(lpath_data.end(), rpath.point.begin(), rpath.point.end());
        lpath_data.insert(lpath_data.end(), lpath.value().point.begin(), lpath.value().point.end());
      }

      last_point = rpath.point;
    }

    m_render_data.length_data.push_back(rpaths.size());
    m_render_data.mask_data.push_back(false);
    m_render_data.dirty = true;
  }

private:
  struct RenderData {
    bool dirty = false;
    std::vector<std::vector<float>> normal_data;
    std::vector<std::vector<float>> rpath_data;
    std::vector<std::vector<float>> lpath_data;

    // ImGui data
    bool fluoroscopy = false;

    bool draw_normal = true;
    bool draw_rpath = true;
    bool draw_lpath = true;

    float line_width = 1.0F;
    std::array<float, 4> normal_color{0.22F, 0.78F, 0.78F, 0.22F};
    std::array<float, 4> rpath_color{0.78F, 0.22F, 0.78F, 0.22F};
    std::array<float, 4> lpath_color{0.78F, 0.78F, 0.78F, 0.22F};

    int filter_range_min = 0;
    int filter_range_max = 0;
    std::vector<int> length_data;
    std::vector<bool> mask_data;
  };

  unsigned int m_image_width;
  RenderData m_render_data;
};

}  // namespace mcpt
