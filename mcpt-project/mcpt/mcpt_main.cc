#include <ctime>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

#include <Eigen/Eigen>
#include <cheers/window/window.hpp>
#include <spdlog/fmt/fmt.h>
#include <spdlog/fmt/chrono.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>

#include "mcpt/common/assert.hpp"
#include "mcpt/common/fileserver/fileserver.hpp"
#include "mcpt/common/viz/object_layer.hpp"
#include "mcpt/common/viz/path_layer.hpp"
#include "mcpt/misc/dispatcher.hpp"
#include "mcpt/parser/obj_parser/parser.hpp"
#include "mcpt/renderer/bxdf.hpp"
#include "mcpt/renderer/monte_carlo.hpp"

#include "mcpt/misc.hpp"

using namespace mcpt;

static const Eigen::IOFormat FMT{
    Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ", "", "", "[", "]"};

Object LoadObject(std::filesystem::path obj_path) {
  ASSERT(!obj_path.empty());
  return obj_parser::Parser(obj_path).object();
}

int main(int argc, char* argv[]) {
  MakeLogger("default");

  ASSERT(argc == 6);
  std::filesystem::path arg_obj_fname = argv[1];
  ASSERT(arg_obj_fname.has_parent_path());
  ASSERT(arg_obj_fname.has_filename());

  unsigned int arg_w = std::stoul(argv[2]);
  unsigned int arg_h = std::stoul(argv[3]);
  ASSERT(arg_w > 0 && arg_h > 0);

  size_t arg_spp = std::stoul(argv[4]);
  size_t arg_spp_save_interval = std::stoul(argv[5]);
  ASSERT(0 < arg_spp_save_interval && arg_spp_save_interval <= arg_spp);

  spdlog::info("loading object from {}", arg_obj_fname);
  SandboxFileserver fserver(arg_obj_fname.parent_path());
  auto obj = LoadObject(fserver.GetAbsolutePath(arg_obj_fname));
  auto bvh = obj.CreateBVHTree();

  spdlog::info("making MCPT options");
  MonteCarlo::Options mc_opts;
  mc_opts.intrin = MakeCamera(arg_w, arg_h, 28.0F, 36.0F);
  mc_opts.R.col(0) = Eigen::Vector3f::UnitX();
  mc_opts.R.col(1) = -Eigen::Vector3f::UnitY();
  mc_opts.R.col(2) = -Eigen::Vector3f::UnitZ();
  mc_opts.t << 8.0F, 8.0F, 18.0F;
  mc_opts.R = Eigen::AngleAxisf(20.0F / 180.0F * M_PI, Eigen::Vector3f::UnitY()) *
              Eigen::AngleAxisf(-10.0F / 180.0F * M_PI, Eigen::Vector3f::UnitX()) * mc_opts.R;
  spdlog::info("camera intrinsics: {}", mc_opts.intrin.format(FMT));

  spdlog::info("making MCPT renderer");
  auto mcpt_runner = std::make_shared<MonteCarlo>(mc_opts, obj, bvh);
  mcpt_runner->SetBxDF(std::make_unique<BlinnPhongBxDF>());

  auto object_layer = cheers::Window::Instance().InstallSharedLayer<ObjectLayer>();
  auto path_layer = cheers::Window::Instance().InstallSharedLayer<PathLayer>(arg_w);
  auto viz_thread = std::thread{VizThread{mc_opts.t * 2.0F}};
  object_layer->UpdateObject(obj, mc_opts.R, mc_opts.t);

  auto t = fmt::localtime(std::time(nullptr));
  auto export_dir =
      fmt::format("out/{:%Y%m%d_%H%M%S}_{}_{}x{}", t, arg_obj_fname.stem().string(), arg_w, arg_h);
  SandboxFileserver fs_out(export_dir);
  spdlog::info("results will be saved in {}", fs_out.GetAbsolutePath());

  unsigned int num_threads = std::thread::hardware_concurrency();
  Dispatcher dispatcher(fs_out, num_threads, arg_spp, arg_spp_save_interval);

  spdlog::info("running MCPT for spp: {}", arg_spp);
  dispatcher.Dispatch(mcpt_runner, path_layer, arg_w, arg_h);
  dispatcher.JoinAll();

  spdlog::info("saved results in {}", fs_out.GetAbsolutePath());
  viz_thread.join();
  return 0;
}
