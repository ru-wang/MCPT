#include <ctime>
#include <filesystem>
#include <memory>
#include <thread>

#include <Eigen/Eigen>
#include <spdlog/fmt/fmt.h>
#include <spdlog/fmt/chrono.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>

#include "mcpt/common/assert.hpp"
#include "mcpt/common/fileserver/fileserver.hpp"
#include "mcpt/misc/argparsing.hpp"
#include "mcpt/misc/dispatcher.hpp"
#include "mcpt/misc/logging.hpp"
#include "mcpt/misc/visualizing.hpp"
#include "mcpt/parser/obj_parser/parser.hpp"
#include "mcpt/renderer/bxdf.hpp"
#include "mcpt/renderer/monte_carlo.hpp"

using namespace mcpt;

static const Eigen::IOFormat FMT{
    Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ", "", "", "[", "]"};

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

int main(int argc, char* argv[]) {
  auto args = misc::InitArgParser("mcpt_main", argc, argv);

  auto launch_time = fmt::localtime(std::time(nullptr));
  auto scene_name = args.scene_path.stem().string();
  auto export_root =
      args.output_path /
      fmt::format("{:%Y%m%d_%H%M%S}_{}_{}x{}", launch_time, scene_name, args.width, args.height);

  SandboxFileserver fs_out(export_root);
  misc::InitLogger("mcpt_main", export_root, args.enable_verbose);
  spdlog::info("results will be saved in {}", fs_out.GetAbsolutePath());

  spdlog::info("loading object from {}", args.scene_path);
  SandboxFileserver fserver(args.scene_path.parent_path());
  auto obj = LoadObject(fserver.GetAbsolutePath(args.scene_path));
  auto bvh = obj.CreateBVHTree();

  spdlog::info("making MCPT options");
  MonteCarlo::Options mc_opts;
  mc_opts.intrin = MakeCamera(args.width, args.height, 28.0F, 36.0F);
  mc_opts.R.col(0) = Eigen::Vector3f::UnitX();
  mc_opts.R.col(1) = -Eigen::Vector3f::UnitY();
  mc_opts.R.col(2) = -Eigen::Vector3f::UnitZ();
  // for scene01
  mc_opts.t << 0.0F, 5.0F, 15.0F;
  // for scene02
  // mc_opts.R = Eigen::AngleAxisf(-25.0F / 180.0F * M_PI, Eigen::Vector3f::UnitX()) * mc_opts.R;
  // mc_opts.t << 2.0F, 9.0F, 16.0F;
  spdlog::info("camera intrinsics: {}", mc_opts.intrin.format(FMT));

  auto viz = misc::InitVisualizer(args.enable_gui, args.width);
  viz.object_layer->UpdateObject(obj, mc_opts.R, mc_opts.t);

  spdlog::info("making MCPT renderer");
  auto mcpt_runner = std::make_shared<MonteCarlo>(mc_opts, obj, bvh);
  mcpt_runner->SetBxDF(std::make_unique<BlinnPhongBxDF>());

  unsigned int num_threads = std::thread::hardware_concurrency();
  Dispatcher dispatcher(fs_out, num_threads, args.spp, args.save_every_n);

  spdlog::info("running MCPT for spp: {}", args.spp);
  dispatcher.Dispatch(mcpt_runner, viz.path_layer, args.width, args.height);

  viz.Run(mc_opts.t.x() * 2.0F, mc_opts.t.y() * 2.0F, mc_opts.t.z() * 2.0F);

  dispatcher.JoinAll();
  spdlog::info("saved results in {}", fs_out.GetAbsolutePath());
  return 0;
}
