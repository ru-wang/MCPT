#pragma once

#include <functional>
#include <string_view>

#include "mcpt/parser/mtl_parser/context.hpp"

namespace mcpt::mtl_parser {

class Tokenizer {
public:
  explicit Tokenizer(std::reference_wrapper<Context> ctx) : m_ctx(ctx) {}

  void Process(std::string_view statement);

private:
  void Proc(std::string_view identifier, std::string_view declaration);

  template <typename T>
  void ProcNumber(std::string_view token, T& number);

  template <size_t Size, typename T>
  void ProcNumericVector(std::string_view tokens, T* vector);

private:
  auto& ctx() noexcept { return m_ctx.get(); }

  std::reference_wrapper<Context> m_ctx;
};

}  // namespace mcpt::mtl_parser
