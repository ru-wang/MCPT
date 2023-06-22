#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <spdlog/cfg/env.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/spdlog.h>

#include "mcpt/common/assert.hpp"
#include "mcpt/common/fileserver/fileserver.hpp"
#include "mcpt/parser/misc.hpp"
#include "mcpt/parser/obj_parser/parser.hpp"
#include "mcpt/renderer/monte_carlo.hpp"

using namespace mcpt;

Object LoadObject(std::filesystem::path obj_path) {
  ASSERT(!obj_path.empty());
  return obj_parser::Parser(obj_path).object();
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
Eigen::Vector4f MakeCamera(unsigned int w, unsigned int h, float focal_length, float cmos_horizon) {
  float cmos_vertical = cmos_horizon * h / w;
  float fx = w * focal_length / cmos_horizon;
  float fy = h * focal_length / cmos_vertical;
  return Eigen::Vector4f(fx, fy, w / 2.0F, h / 2.0F);
}

void MakeLogger() {
  auto console_sink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("log.txt", true);
  auto logger = std::make_shared<spdlog::logger>(
      "default", spdlog::sinks_init_list({console_sink, file_sink}));
  spdlog::set_default_logger(logger);

  spdlog::set_pattern(R"([%H:%M:%S.%e %L%t] %^%v%$)");
  spdlog::cfg::load_env_levels();
}

int main(int argc, char* argv[]) {
  MakeLogger();

  ASSERT(argc == 5);
  std::filesystem::path asset_root = argv[1];
  std::filesystem::path obj_filename = argv[2];
  unsigned int w = std::stoul(argv[3]);
  unsigned int h = std::stoul(argv[4]);

  spdlog::info("loading object from {}", obj_filename);
  SandboxFileserver fsserver(asset_root);
  auto obj = LoadObject(fsserver.GetAbsolutePath(obj_filename));

  spdlog::info("making MCPT renderer");
  MonteCarlo::options options;
  options.intrin = MakeCamera(w, h, 28.0F, 36.0F);
  options.R.col(0) = Eigen::Vector3f::UnitX();
  options.R.col(1) = -Eigen::Vector3f::UnitZ();
  options.R.col(2) = Eigen::Vector3f::UnitY();
  options.t << 0.0F, -200.0F, 50.0F;
  MonteCarlo mcpt(options, obj);

  spdlog::info("running MCPT configs");
  std::vector<float> im(w * h * 3);
  for (unsigned int v = 0, px = 0; v < h; ++v) {
    for (unsigned int u = 0; u < w; ++u, ++px) {
      auto color = mcpt.Run(u, v);
      im.at(px * 3 + 0) = color.x();
      im.at(px * 3 + 1) = color.y();
      im.at(px * 3 + 2) = color.z();
    }
  }

  auto ppm_filepath = obj_filename.replace_extension(".ppm");
  spdlog::info("saving to PPM image {}", ppm_filepath);

  std::ofstream ofs;
  ASSERT(fsserver.OpenTextWrite(ppm_filepath, ofs));
  SaveRawToPPM(w, h, im.data(), ofs);
  return 0;
}
