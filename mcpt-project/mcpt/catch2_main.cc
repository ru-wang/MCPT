#define CATCH_CONFIG_RUNNER

#include <catch2/catch_all.hpp>

#include "mcpt/misc/logging.hpp"

int main(int argc, char* argv[]) {
  mcpt::misc::InitLogger("test", ".", false);
  return Catch::Session().run(argc, argv);
}
