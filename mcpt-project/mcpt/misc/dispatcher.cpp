#include "mcpt/misc/dispatcher.hpp"

#include <atomic>
#include <fstream>
#include <mutex>
#include <utility>

#include <Eigen/Eigen>
#include <spdlog/fmt/fmt.h>
#include <spdlog/fmt/chrono.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include "mcpt/common/assert.hpp"
#include "mcpt/common/misc.hpp"

namespace {

void reduce(const std::vector<Eigen::Vector3f>& radiance, std::vector<float>& integral) {
  integral.resize(radiance.size() * 3, 0.0F);
  for (size_t i = 0; i < radiance.size(); ++i) {
    integral[i * 3 + 0] += radiance[i].x();
    integral[i * 3 + 1] += radiance[i].y();
    integral[i * 3 + 2] += radiance[i].z();
  }
}

std::future<void> save(const std::vector<float>& integral,
                       mcpt::Fileserver& fs_out,
                       size_t spp,
                       unsigned int width,
                       unsigned int height) {
  std::vector<float> im(integral.size());
  for (size_t i = 0; i < integral.size(); ++i)
    im[i] = integral[i] / spp;

  return std::async(std::launch::async, [=, &fs_out, im = std::move(im)]() noexcept {
    auto export_name = fmt::format("spp_{}.ppm", spp);
    spdlog::info("saving to PPM image {}", fs_out.GetAbsolutePath(export_name));

    std::ofstream ofs;
    ASSERT(fs_out.OpenTextWrite(export_name, ofs));
    mcpt::RawToPPM(width, height, 2.2F, im.data(), ofs);
  });
}

}  // namespace

void Dispatcher::Dispatch(const std::shared_ptr<mcpt::MonteCarlo>& mcpt_runner,
                          const std::shared_ptr<mcpt::PathLayer>& path_layer,
                          unsigned int width,
                          unsigned int height) {
  static std::atomic_size_t shared_spp_idx = 0;

  static std::mutex mutex;
  static size_t spp_completed = 0;
  static std::vector<float> integral;
  static spdlog::stopwatch sw;

  auto worker = [=]() {
    std::vector<Eigen::Vector3f> radiance(width * height);

    for (size_t spp_idx = ++shared_spp_idx; spp_idx <= m_spp; spp_idx = ++shared_spp_idx) {
      spdlog::stopwatch sw_spp;

      for (size_t i = 0; i < width * height; ++i) {
        auto result = mcpt_runner->Run(i % width, i / width);
        radiance[i].noalias() = result.radiance;
        if (spp_idx == 1 && !result.rpaths.empty())
          path_layer->AddPaths(mcpt_runner->options().t, result.rpaths);
      }

      spdlog::info(
          "spp: {}/{}, {:%M:%Ss} {{{:%M:%Ss}}}", spp_idx, m_spp, sw_spp.elapsed(), sw.elapsed());

      {
        std::lock_guard lock(mutex);
        reduce(radiance, integral);
        ++spp_completed;
        if (spp_completed == m_spp || (m_save_every_n && spp_completed % m_save_every_n == 0))
          m_saving_tasks.push_back(save(integral, m_fs_out, spp_completed, width, height));
      }
    }
  };

  for (unsigned int i = 0; i < m_spp && i < m_num_threads; ++i)
    m_worker_threads.emplace_back(worker);
}

void Dispatcher::JoinAll() {
  for (auto& t : m_worker_threads) {
    ASSERT(t.joinable());
    t.join();
  }
  for (auto& t : m_saving_tasks) {
    ASSERT(t.valid());
    t.get();
  }
}
