#define CATCH_CONFIG_RUNNER

#include <catch2/catch.hpp>
#include <spdlog/cfg/env.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/spdlog.h>

void MakeLogger() {
  auto console_sink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("log.txt", true);
  auto logger =
      std::make_shared<spdlog::logger>("test", spdlog::sinks_init_list({console_sink, file_sink}));
  spdlog::set_default_logger(logger);

  spdlog::set_pattern(R"([%H:%M:%S.%e %L%t] %^%v%$)");
  spdlog::cfg::load_env_levels();
}

int main(int argc, char* argv[]) {
  MakeLogger();
  return Catch::Session().run(argc, argv);
}
