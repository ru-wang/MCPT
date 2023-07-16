#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Eigen>
#include <cheers/window/window.hpp>
#include <spdlog/cfg/env.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/spdlog.h>

namespace mcpt {

inline void MakeLogger(std::string name) {
  auto console_sink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(name + "_log.txt", true);
  auto logger =
      std::make_shared<spdlog::logger>(name, spdlog::sinks_init_list({console_sink, file_sink}));
  spdlog::set_default_logger(logger);

  spdlog::set_pattern(R"([%H:%M:%S.%e %L%t] %^%v%$)");
  spdlog::cfg::load_env_levels();
}

/**
 * Y(m) |\
 *      | \
 *      |  \
 *      |   \
 *      |    \       /| y(mm): vertical CMOS half size
 *      |     \     / |
 *      |      \   /  |
 *      |       \ /   |
 *    --+--------X----+--
 *         d(m)    f(mm)
 *
 * Y/d = y/f = fy
 *
 * h/2 = fy * Y/d
 *  fy = h/2 * d/Y
 *     = h/2 * f/y
 */
inline Eigen::Vector4f MakeCamera(unsigned int w,
                                  unsigned int h,
                                  float focal_length,
                                  float cmos_horizon) {
  float cmos_vertical = cmos_horizon * h / w;
  float fx = w * focal_length / cmos_horizon;
  float fy = h * focal_length / cmos_vertical;
  return Eigen::Vector4f(fx, fy, w / 2.0F, h / 2.0F);
}

struct VizThread {
  Eigen::Vector3f eye;

  void operator()() const noexcept {
    cheers::Window::Instance().CreateContext();
    cheers::Window::Instance().SetInitEyeAndUp(eye.x(), eye.y(), eye.z(), 0.0F, 1.0F, 0.0F);
    while (cheers::Window::Instance().WaitForWindowExiting())
      ;
    cheers::Window::Instance().DestroyContext();
  }
};

}  // namespace mcpt
