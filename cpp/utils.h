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
  static bool Equal(double a, double b) {
    return std::fabs(a - b) < kEpsilon;
  }

  static bool IsZero(double a) {
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

  static void SaveRGBToPPM(const double* image, double n, int w, int h, const std::string& str) {
    static constexpr double kKernal[9] = {0.0625, 0.125, 0.0625,
                                          0.125,  0.25,  0.125,
                                         0.0625, 0.125, 0.0625};

    std::ofstream ofs(str);
    ofs << "P3\n" << w << " " << h << " 255\n";

    double* before_image = new double[w * h * 3]{0};
    double* after_image = new double[w * h * 3]{0};

#pragma omp parallel for
    for (int i = 0; i < w * h; ++i) {
      double& r = before_image[i * 3 + 0] = image[i * 3 + 0] * 255 / n;
      double& g = before_image[i * 3 + 1] = image[i * 3 + 1] * 255 / n;
      double& b = before_image[i * 3 + 2] = image[i * 3 + 2] * 255 / n;

      if (r > 255 && r >= g && r >= b) {
        g *= (255 / r);
        b *= (255 / r);
        r = 255;
      } else
      if (g > 255 && g >= r && g >= b) {
        r *= (255 / g);
        b *= (255 / g);
        g = 255;
      } else
      if (b > 255 && b >= r && b >= g) {
        r *= (255 / b);
        g *= (255 / b);
        b = 255;
      }
    }

    /* Gaussian filter */
#pragma omp parallel for
    for (int i = 1; i < h - 1; ++i) {
      for (int j = 1; j < w - 1; ++j) {
        int p = i * w + j;
        for (int ki = -1; ki < 2; ++ki) {
          for (int kj = -1; kj < 2; ++kj) {
            int pk = (i + ki) * w + j + kj;
            int kk = (1 + ki) * 3 + (1 + kj);
            after_image[p * 3 + 0] += before_image[pk * 3 + 0] * kKernal[kk];
            after_image[p * 3 + 1] += before_image[pk * 3 + 1] * kKernal[kk];
            after_image[p * 3 + 2] += before_image[pk * 3 + 2] * kKernal[kk];
          }
        }
      }
    }

    for (int i = 0; i < w * h; ++i) {
      ofs << std::setw(8) << (int)after_image[i * 3 + 0] << " "
          << std::setw(8) << (int)after_image[i * 3 + 1] << " "
          << std::setw(8) << (int)after_image[i * 3 + 2];
      if ((i + 1) % w == 0)
        ofs << "\n";
      else
        ofs << " ";
    }

    delete [] before_image, before_image = nullptr;
    delete [] after_image, after_image = nullptr;
    ofs.close();
  }

  static constexpr double Epsilon() { return kEpsilon; }

 private:
  static constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

  Utils() = delete;
  ~Utils() = delete;
  Utils(const Utils&) = delete;
  Utils& operator=(const Utils&) = delete;
};

#endif  /* MCPT_UTILS_H_ */
