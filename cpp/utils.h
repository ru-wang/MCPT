#ifndef MCPT_UTILS_H_
#define MCPT_UTILS_H_

#include <cmath>
#include <fstream>
#include <iomanip>
#include <istream>
#include <limits>
#include <sstream>
#include <streambuf>
#include <string>

class Utils {
 public:
  static bool Equal(float a, float b) {
    return std::fabs(a - b) < kEpsilon;
  }

  static bool IsZero(float a) {
    return std::fabs(a) < kEpsilon;
  }

  static std::string RemoveSuffix(const std::string& str, const char delimter = '.') {
    auto rit = str.crbegin();
    for (; rit != str.crend(); ++rit) {
      if (*rit == delimter)
        break;
    }
    std::stringstream ss;
    for (auto it = str.crend(); it != rit; --it) {
      if (*it)
        ss << *it;
    }
    return ss.str();
  }

  static std::stringstream SafelyGetLine(std::istream& is) {
    std::stringstream ss;
    std::streambuf* sb = is.rdbuf();
    while (true) {
      char ch = sb->sbumpc();
      switch (ch) {
        case '\n': return ss;
        case '\r': if (sb->sgetc() == '\n')
                     sb->sbumpc();
                   return ss;
        case EOF:  if (ss.rdbuf()->in_avail() == 0)
                     is.setstate(std::istream::eofbit);
                   return ss;
        default:   ss << ch;
      }
    }
  }

  static void SaveRGBToPPM(const float* image, int w, int h, const std::string& str) {
    std::ofstream ofs(str);
    ofs << "P3\n" << w << " " << h << " 255\n";

    int r, g, b;
    for (int i = 0; i < w * h; ++i) {
      r = (int)image[i * 3 + 0] * 255;
      g = (int)image[i * 3 + 1] * 255;
      b = (int)image[i * 3 + 2] * 255;

      r = r < 0 ? 0 : (r > 255 ? 255 : r);
      g = g < 0 ? 0 : (g > 255 ? 255 : g);
      b = b < 0 ? 0 : (b > 255 ? 255 : b);

      ofs << std::setw(3) << r << " " << std::setw(3) << g << " " << std::setw(3) << b;
      if ((i + 1) % w == 0)
        ofs << "\n";
      else
        ofs << " ";
    }

    ofs.close();
  }

 private:
  static constexpr float kEpsilon = std::numeric_limits<float>::epsilon();

  Utils() = delete;
  ~Utils() = delete;
  Utils(const Utils&) = delete;
  Utils& operator=(const Utils&) = delete;
};

#endif  /* MCPT_UTILS_H_ */
