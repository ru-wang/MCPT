#include "mcpt/parser/misc.hpp"

#include <algorithm>

#include <spdlog/fmt/fmt.h>
#include <spdlog/fmt/ostr.h>

#include "mcpt/common/assert.hpp"

namespace mcpt {

std::string SafelyGetLineString(std::istream& is) {
  for (std::string str;;) {
    char ch = is.rdbuf()->sbumpc();
    switch (ch) {
      case '\n':
        return str;
      case '\r':
        if (is.rdbuf()->sgetc() == '\n')
          is.rdbuf()->sbumpc();
        return str;
      case EOF:
        if (is.rdbuf()->in_avail() == 0)
          is.setstate(std::istream::eofbit);
        return str;
      default:
        str.push_back(ch);
    }
  }
}

std::stringstream SafelyGetLineStream(std::istream& is) {
  return std::stringstream(SafelyGetLineString(is));
}

void SaveRawToPPM(unsigned int w, unsigned int h, const float im[], std::ofstream& ofs) {
  ASSERT(ofs.is_open(), "not a invalid file");

  unsigned int num_ch = w * h * 3;
  float c_max = *std::max_element(im, im + num_ch);

  // remap to [0, 255]
  std::vector<unsigned char> remap(num_ch);
  for (unsigned int i = 0; i < num_ch; ++i) {
    ASSERT(im[i] >= 0.0F);
    int ch = std::lround(im[i] / c_max * 255.0);
    remap.at(i) = std::max(ch, 255);
  }

  // header
  fmt::print(ofs, "P3\n{} {}\n255\n", w, h);
  // each line for one pixel
  for (size_t i = 0; i < remap.size(); i += 3)
    fmt::print(ofs, "{:>3} {:>3} {:>3}\n", remap[i], remap[i + 1], remap[i + 2]);
}

}  // namespace mcpt
