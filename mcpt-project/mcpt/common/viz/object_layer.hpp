#pragma once

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

    // generated
    size_t num_indices = 0;
  };

  RenderData m_render_data;
};

}  // namespace mcpt
