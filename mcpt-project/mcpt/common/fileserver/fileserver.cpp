#include "mcpt/common/fileserver/fileserver.hpp"

#include <spdlog/spdlog.h>

#include "mcpt/common/assert.hpp"

namespace mcpt {

namespace fs = std::filesystem;

SandboxFileserver::SandboxFileserver(const fs::path& root_path) {
  // compose absolute path lexically
  m_root_path = fs::current_path() / root_path;
  // assert exists and is a directory
  ASSERT(fs::is_directory(m_root_path), "not a valid directory: {}", root_path);
  // remove dots and symlinks
  m_root_path = fs::canonical(m_root_path);
}

std::filesystem::path SandboxFileserver::GetAbsolutePath(const std::filesystem::path& relpath) {
  if (!relpath.has_filename()) {
    spdlog::error("failed to get absolute path {}: not a valid file path", relpath);
    return {};
  }

  // compose canonical path lexically
  fs::path canonical_relpath = relpath.lexically_normal();

  // allow symlinks to out the sandbox
  fs::path abspath = fs::canonical(m_root_path / canonical_relpath);
  if (fs::exists(abspath) && !fs::is_regular_file(abspath)) {
    spdlog::error("failed to get absolute path {}: not a regular file", relpath);
    return {};
  }
  return abspath;
}

bool SandboxFileserver::OpenTextForRead(const std::filesystem::path& relpath, std::ifstream& ifs) {
  if (ifs.is_open()) {
    spdlog::error("failed to open {}: file stream aleady occupied", relpath);
    return false;
  }

  auto abspath = GetAbsolutePath(relpath);
  if (abspath.empty())
    return false;

  ifs.open(abspath);
  if (ifs.is_open()) {
    spdlog::error("failed to open {}", relpath);
    return false;
  }
  return true;
}

bool SandboxFileserver::OpenTextWrite(const std::filesystem::path& relpath, std::ofstream& ofs) {
  if (ofs.is_open()) {
    spdlog::error("failed to open {}: file stream aleady occupied", relpath);
    return false;
  }

  auto abspath = GetAbsolutePath(relpath);
  if (abspath.empty())
    return false;

  ofs.open(abspath);
  if (ofs.is_open()) {
    spdlog::error("failed to open {}", relpath);
    return false;
  }
  return true;
}

}  // namespace mcpt
