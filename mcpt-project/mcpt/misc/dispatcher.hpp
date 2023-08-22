#pragma once

#include <deque>
#include <functional>
#include <future>
#include <memory>
#include <thread>
#include <vector>

#include "mcpt/common/fileserver/fileserver.hpp"
#include "mcpt/common/viz/path_layer.hpp"
#include "mcpt/renderer/monte_carlo.hpp"

class Dispatcher {
public:
  Dispatcher(mcpt::Fileserver& fs_out, unsigned int num_threads, size_t spp, size_t save_every_n)
      : m_fs_out(fs_out), m_num_threads(num_threads), m_spp(spp), m_save_every_n(save_every_n) {}

  void Dispatch(const std::shared_ptr<mcpt::MonteCarlo>& mcpt_runner,
                const std::shared_ptr<mcpt::PathLayer>& path_layer,
                unsigned int width,
                unsigned int height);

  void JoinAll();

private:
  std::reference_wrapper<mcpt::Fileserver> m_fs_out;
  unsigned int m_num_threads;
  size_t m_spp;
  size_t m_save_every_n;

  std::vector<std::thread> m_worker_threads;
  std::deque<std::future<void>> m_saving_tasks;
};
