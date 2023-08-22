#include "mcpt/misc/argparsing.hpp"

#include <cstdlib>
#include <exception>
#include <iostream>

namespace mcpt::misc {

namespace {

template <typename T>
T Get(const argparse::ArgumentParser& parser,
      const std::string& name,
      const std::function<bool(const T&)>& validate = {}) {
  auto val = parser.get<T>(name);
  if (validate && !validate(val)) {
    std::cerr << "invalid argument for `" << name << "': " << val;
    std::cerr << parser;
  }
  return val;
}

}  // namespace

RuntimeArgs InitArgParser(const std::string& name, int argc, char* argv[]) {
  argparse::ArgumentParser parser(name, "1.0", argparse::default_arguments::help);

  parser.add_argument("scene").required().help("path to the scene object (.obj)").metavar("SCENE");
  parser.add_argument("width")
      .required()
      .help("output image width")
      .metavar("WIDTH")
      .scan<'u', unsigned int>();
  parser.add_argument("height")
      .required()
      .help("output image height")
      .metavar("HEIGHT")
      .scan<'u', unsigned int>();

  parser.add_argument("-s", "--spp")
      .required()
      .help("samples per pixel")
      .metavar("SAPMLES_PER_PIXEL")
      .scan<'u', unsigned int>();
  parser.add_argument("-n", "--save-every-n")
      .help("save intermediate results every N spp (zero means don't save)")
      .metavar("N")
      .default_value(0)
      .scan<'u', unsigned int>();

  parser.add_argument("-o", "--output")
      .help("output root directory")
      .metavar("OUTPUT")
      .default_value(std::string("./out"));
  parser.add_argument("-v", "--verbose")
      .help("enable verbose logging")
      .default_value(false)
      .implicit_value(true);
  parser.add_argument("-g", "--gui")
      .help("enable GUI visualization")
      .default_value(false)
      .implicit_value(true);

  parser.add_description("Monte Carlo path tracing renderer.");

  try {
    parser.parse_args(argc, argv);
  } catch (const std::runtime_error& err) {
    std::cerr << err.what() << std::endl;
    std::cerr << parser;
  }

  RuntimeArgs args;
  args.scene_path = Get<std::string>(parser, "scene", [](auto& v) {
    std::filesystem::path p(v);
    return p.has_parent_path() && p.has_filename();
  });
  args.width = Get<unsigned int>(parser, "width", [](auto v) { return v > 0; });
  args.height = Get<unsigned int>(parser, "height", [](auto v) { return v > 0; });

  args.spp = Get<unsigned int>(parser, "-s", [](auto v) { return v > 0; });
  args.save_every_n =
      Get<unsigned int>(parser, "-n", [spp = args.spp](auto v) { return v <= spp; });

  args.output_path = Get<std::string>(parser, "-o");
  args.enable_gui = Get<bool>(parser, "-g");
  args.enable_verbose = Get<bool>(parser, "-v");

  return args;
}

}  // namespace mcpt::misc
