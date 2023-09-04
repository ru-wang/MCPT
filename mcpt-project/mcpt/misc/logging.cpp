#include "mcpt/misc/logging.hpp"

#include <memory>

#include <spdlog/cfg/env.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/spdlog.h>

namespace mcpt::misc {

void InitLogger(const std::string& name, const std::filesystem::path& logdir, bool verbose) {
#ifndef NDEBUG
  // create console sink
  auto console_sink = std::make_shared<spdlog::sinks::stderr_sink_mt>();
#else
  // create colorful console sink
  auto console_sink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
#endif
  // create file sink
  auto logfile = logdir / std::filesystem::path(name).replace_extension("log");
  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(logfile, true);
  // set default logger with multiple sinks and configure
  spdlog::set_default_logger(
      std::make_shared<spdlog::logger>(name, spdlog::sinks_init_list{console_sink, file_sink}));

  // following settings will be applied to all loggers
  spdlog::set_pattern(R"([%H:%M:%S.%e %L%t] %^%v%$)");
  // first load config from environment variables
  spdlog::cfg::load_env_levels();
  // verbose flag will override the environment variables
  if (verbose)
    spdlog::set_level(spdlog::level::debug);
}

}  // namespace mcpt::misc
