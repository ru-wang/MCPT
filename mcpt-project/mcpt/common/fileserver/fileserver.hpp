#pragma once

#include <filesystem>
#include <fstream>

namespace mcpt {

class Fileserver {
public:
  virtual ~Fileserver() = default;
  virtual std::filesystem::path GetAbsolutePath() = 0;
  virtual std::filesystem::path GetAbsolutePath(const std::filesystem::path& path) = 0;
  virtual bool OpenTextForRead(const std::filesystem::path& path, std::ifstream& ifs) = 0;
  virtual bool OpenTextWrite(const std::filesystem::path& path, std::ofstream& ofs) = 0;
};

class SandboxFileserver : public Fileserver {
public:
  SandboxFileserver() : SandboxFileserver(std::filesystem::current_path()) {}
  explicit SandboxFileserver(const std::filesystem::path& root_path);
  ~SandboxFileserver() override = default;

  std::filesystem::path GetAbsolutePath() override;
  std::filesystem::path GetAbsolutePath(const std::filesystem::path& relpath) override;
  bool OpenTextForRead(const std::filesystem::path& relpath, std::ifstream& ifs) override;
  bool OpenTextWrite(const std::filesystem::path& relpath, std::ofstream& ofs) override;

private:
  std::filesystem::path m_root_path;
};

}  // namespace mcpt
