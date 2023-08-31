#pragma once

#include <filesystem>
#include <string>

namespace mcpt::misc {

void InitLogger(const std::string& name, const std::filesystem::path& logdir, bool verbose);

}  // namespace mcpt::misc
