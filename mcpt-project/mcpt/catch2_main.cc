#define CATCH_CONFIG_RUNNER

#include <catch2/catch_all.hpp>

#include "mcpt/misc.hpp"

int main(int argc, char* argv[]) {
  mcpt::MakeLogger("test");
  return Catch::Session().run(argc, argv);
}
