#include "mcpt/common/misc.hpp"

#include <cmath>
#include <algorithm>
#include <numeric>

#include <spdlog/fmt/fmt.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>

#include "mcpt/common/assert.hpp"

namespace mcpt {

std::string SafelyGetLineString(std::istream& is) {
  for (std::string str;;) {
    char ch = is.rdbuf()->sbumpc();
    switch (ch) {
      case '\n': return str;
      case '\r':
        if (is.rdbuf()->sgetc() == '\n')
          is.rdbuf()->sbumpc();
        return str;
      case EOF:
        if (is.rdbuf()->in_avail() == 0)
          is.setstate(std::istream::eofbit);
        return str;
      default: str.push_back(ch);
    }
  }
}

std::stringstream SafelyGetLineStream(std::istream& is) {
  return std::stringstream(SafelyGetLineString(is));
}

void DepthToPPM(unsigned int w, unsigned int h, const float im[], std::ofstream& ofs) {
  ASSERT(ofs.is_open(), "not a invalid file");
  auto [d_min, d_max] = std::minmax_element(im, im + w * h);
  float d_avg = std::accumulate(im, im + w * h, 0.0F) / (w * h);
  spdlog::info("min depth: {}, max depth: {}, avg depth: {}", *d_min, *d_max, d_avg);

  // remap to [0, 255]
  std::vector<unsigned char> remap(w * h);
  for (size_t i = 0; i < remap.size(); ++i) {
    ASSERT(im[i] >= 0.0F, "wrong depth value: ({},{}) {}", i % w, i / w, im[i]);
    float val = (im[i] - *d_min) / (*d_max);
    remap.at(i) = std::clamp<unsigned int>(val * 255.0F, 0, 255);
  }

  // header
  fmt::print(ofs, "P3\n{} {}\n255\n", w, h);
  // each line for one pixel
  for (size_t i = 0; i < remap.size(); ++i)
    fmt::print(ofs, "{:>3} {:>3} {:>3}\n", remap[i], remap[i], remap[i]);
}

void RawToPPM(unsigned int w, unsigned int h, float gamma, const float im[], std::ofstream& ofs) {
  ASSERT(ofs.is_open(), "not a invalid file");
  float c_max = *std::max_element(im, im + w * h * 3);
  float c_avg = std::accumulate(im, im + w * h * 3, 0.0F) / (w * h * 3);
  spdlog::info("max channel: {}, avg channel: {}", c_max, c_avg);

  // remap to [0, 255]
  std::vector<unsigned char> remap(w * h * 3);
  for (size_t i = 0; i < remap.size(); ++i) {
    size_t u = i / 3 % w;
    size_t v = i / 3 / w;
    size_t ch = i % 3;

    ASSERT(im[i] >= 0.0F, "wrong channel value: ({},{},{}) {}", u, v, ch, im[i]);
    float val = std::pow(std::clamp(im[i], 0.0F, 1.0F), 1.0F / gamma);
    remap.at(i) = std::clamp<unsigned int>(std::round(val * 255.0F), 0, 255);
  }

  // header
  fmt::print(ofs, "P3\n{} {}\n255\n", w, h);
  // each line for one pixel
  for (size_t i = 0; i < remap.size(); i += 3)
    fmt::print(ofs, "{:>3} {:>3} {:>3}\n", remap[i], remap[i + 1], remap[i + 2]);
}

}  // namespace mcpt
