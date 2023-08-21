#include "mcpt/misc/visualizing.hpp"

#include <cheers/window/window.hpp>

namespace mcpt::misc {

void Visualizer::Run() {
  if (enable) {
    cheers::Window::Instance().CreateContext();
    while (cheers::Window::Instance().WaitForWindowExiting())
      ;
    cheers::Window::Instance().DestroyContext();
  }
}

void Visualizer::Run(float eye_x, float eye_y, float eye_z, float up_x, float up_y, float up_z) {
  if (enable) {
    cheers::Window::Instance().CreateContext();
    cheers::Window::Instance().SetInitEyeAndUp(eye_x, eye_y, eye_z, up_x, up_y, up_z);
    while (cheers::Window::Instance().WaitForWindowExiting())
      ;
    cheers::Window::Instance().DestroyContext();
  }
}

Visualizer InitVisualizer(bool enable, unsigned int image_width) {
  Visualizer visualizer{enable};
  visualizer.object_layer = cheers::Window::Instance().InstallSharedLayer<ObjectLayer>();
  visualizer.shape_layer = cheers::Window::Instance().InstallSharedLayer<ShapeLayer>();
  if (image_width)
    visualizer.path_layer = cheers::Window::Instance().InstallSharedLayer<PathLayer>(image_width);
  return visualizer;
}

}  // namespace mcpt::misc
