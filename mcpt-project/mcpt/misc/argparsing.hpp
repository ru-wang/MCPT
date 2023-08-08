#pragma once

#include <string>
#include <filesystem>

#include <argparse/argparse.hpp>

namespace mcpt::misc {

struct RuntimeArgs {
  std::filesystem::path scene_path;
  unsigned int width;
  unsigned int height;
  unsigned int spp;
  unsigned int interval;

  std::filesystem::path output_path;
  bool enable_gui;
  bool enable_verbose;
};

RuntimeArgs InitArgParser(const std::string& name, int argc, char* argv[]);

}  // namespace mcpt::misc
