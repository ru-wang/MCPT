#include "mcpt/common/fileserver/fileserver.hpp"

#include <spdlog/spdlog.h>

#include "mcpt/common/assert.hpp"

namespace mcpt {

namespace fs = std::filesystem;

SandboxFileserver::SandboxFileserver(const fs::path& root_path) {
  // assert exists and is a directory
  ASSERT(fs::is_directory(root_path), "not a valid directory: {}", root_path);
  // convert to aboslute and remove dots and symlinks
  m_root_path = fs::canonical(root_path);
}

std::filesystem::path SandboxFileserver::GetAbsolutePath(const std::filesystem::path& relpath) {
  auto abspath = fs::weakly_canonical(m_root_path / relpath);
  if (!relpath.has_filename()) {
    spdlog::error("failed to get absolute path {} ({}): not a valid file path", relpath, abspath);
    return {};
  }
  return abspath;
}

bool SandboxFileserver::OpenTextForRead(const std::filesystem::path& relpath, std::ifstream& ifs) {
  auto abspath = GetAbsolutePath(relpath);
  if (!fs::is_regular_file(abspath)) {
    spdlog::error("failed to open {} ({}): not a regular file", relpath, abspath);
    return false;
  }

  if (ifs.is_open()) {
    spdlog::error("failed to open {} ({}): file stream aleady occupied", relpath, abspath);
    return false;
  }

  ifs.open(abspath);
  if (!ifs.is_open()) {
    spdlog::error("failed to open {} ({})", relpath, abspath);
    return false;
  }
  return true;
}

bool SandboxFileserver::OpenTextWrite(const std::filesystem::path& relpath, std::ofstream& ofs) {
  auto abspath = GetAbsolutePath(relpath);
  if (fs::exists(abspath) && !fs::is_regular_file(abspath)) {
    spdlog::error("failed to open {} ({}): not a regular file", relpath, abspath);
    return false;
  }

  if (ofs.is_open()) {
    spdlog::error("failed to open {} ({}): file stream aleady occupied", relpath, abspath);
    return false;
  }

  ofs.open(abspath);
  if (!ofs.is_open()) {
    spdlog::error("failed to open {} ({})", relpath, abspath);
    return false;
  }
  return true;
}

}  // namespace mcpt
