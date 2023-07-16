#pragma once

#include <array>
#include <mutex>

#include <Eigen/Eigen>
#include <cheers/layer/layer.hpp>

#include "mcpt/common/object/object.hpp"

namespace mcpt {

class ObjectLayer : public cheers::Layer {
public:
  ~ObjectLayer() noexcept override = default;

  void OnCreateRenderer() override;
  void OnDestroyRenderer() override;

  void OnUpdateImFrame() override;
  void OnUpdateRenderData() override;
  void OnRenderLayer(const float* matrix_vp) override;

  void UpdateObject(const Object& object, const Eigen::Matrix3f& R, const Eigen::Vector3f& t) {
    std::lock_guard lock(render_data_mutex());
    m_render_data.object = object;
    m_render_data.camera_pose.setIdentity();
    m_render_data.camera_pose.topLeftCorner<3, 3>() = R;
    m_render_data.camera_pose.topRightCorner<3, 1>() = t;
    m_render_data.dirty = true;
  }

private:
  struct RenderData {
    bool dirty = false;

    Object object;
    Eigen::Matrix4f camera_pose;

    // ImGui data
    bool draw_wire = true;
    float line_width = 2.0F;
    std::array<float, 4> facet_color{0.22F, 0.22F, 0.22F, 0.78F};
    std::array<float, 4> wire_color{0.1F, 0.1F, 0.1F, 1.0F};
  };

  RenderData m_render_data;
};

}  // namespace mcpt
