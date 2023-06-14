#pragma once

#include <functional>
#include <string_view>

#include "mcpt/parser/obj_parser/context.hpp"

namespace mcpt::obj_parser {

class Tokenizer {
public:
  explicit Tokenizer(std::reference_wrapper<Context> ctx) : m_ctx(ctx) {}

  void Process(std::string_view statement);

private:
  void Proc(std::string_view identifier, std::string_view declaration);

  void ProcMTLFilename(std::string_view mtl_filename);
  void ProcGroupName(std::string_view group_name);

  template <size_t Size, typename T>
  void ProcNumericVector(std::string_view tokens, T* vector);

  void ProcMeshIndex(std::string_view tokens);
  void ProcMaterialDecl(std::string_view material_name);

private:
  auto& ctx() noexcept { return m_ctx.get(); }

  std::reference_wrapper<Context> m_ctx;
};

}  // namespace mcpt::obj_parser
