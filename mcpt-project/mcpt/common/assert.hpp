#pragma once

#include <cstdlib>

#include <spdlog/fmt/fmt.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>

#define STATIC_ASSERT static_assert

#define ASSERT(expr, ...)  \
  (static_cast<bool>(expr) \
       ? void(0)           \
       : mcpt::AssertFail(#expr, __FILE__, __LINE__, __PRETTY_FUNCTION__, ##__VA_ARGS__))

#define ASSERT_FAIL(...) \
  mcpt::AssertFail("this should not happen", __FILE__, __LINE__, __PRETTY_FUNCTION__, ##__VA_ARGS__)

namespace mcpt {

template <typename... Args>
void AssertFail(const char assertion[],
                const char file[],
                unsigned int line,
                const char function[],
                const Args&... args) noexcept {
  spdlog::critical("{}:{}: {}", file, line, function);
  spdlog::critical("  assertion `{}' failed", assertion);
  if constexpr (sizeof...(Args) > 0)
    spdlog::critical("  {}", fmt::format(args...));
  std::abort();
}

}  // namespace mcpt
