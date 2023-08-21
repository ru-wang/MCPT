#pragma once

#include <memory>

#include "mcpt/common/viz/object_layer.hpp"
#include "mcpt/common/viz/path_layer.hpp"
#include "mcpt/common/viz/shape_layer.hpp"

namespace mcpt::misc {

struct Visualizer {
  bool enable = false;
  std::shared_ptr<ObjectLayer> object_layer;
  std::shared_ptr<ShapeLayer> shape_layer;
  std::shared_ptr<PathLayer> path_layer;

  Visualizer(Visualizer&&) = default;

  void Run();
  void Run(float eye_x,
           float eye_y,
           float eye_z,
           float up_x = 0.0F,
           float up_y = 1.0F,
           float up_z = 0.0F);
};

Visualizer InitVisualizer(bool enable, unsigned int image_width = 0);

}  // namespace mcpt::misc
