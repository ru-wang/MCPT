#include "monte_carlo.h"
#include "obj_parser.h"
#include "scene.h"

#include <cassert>
#include <fstream>
#include <iomanip>
#include <string>

using namespace std;

namespace {

string RemoveFileExtension(const string& filename) {
  auto rit = filename.crbegin();
  while (rit != filename.crend()) {
    if (*(rit++) == '.')
      break;
  }
  return string(rit, filename.crend());
}

void SaveRGBToPPM(const float* image, int w, int h, const std::string& filename) {
  ofstream ofs(filename);
  ofs << "P3\n" << w << " " << h << "\n";

  int r, g, b;
  for (int i = 0; i < w * h; ++i) {
    r = (int)image[i * 3 + 0] * 255;
    g = (int)image[i * 3 + 1] * 255;
    b = (int)image[i * 3 + 2] * 255;

    r = r < 0 ? 0 : (r > 255 ? 255 : r);
    g = g < 0 ? 0 : (g > 255 ? 255 : g);
    b = b < 0 ? 0 : (b > 255 ? 255 : b);

    ofs << setw(3) << r << " " << setw(3) << g << " " << setw(3) << b;
    if ((i + 1) % w == 0)
      ofs << "\n";
    else
      ofs << " ";
  }

  ofs.close();
}

const int w = 640, h = 480;
const Matrix4f MVP_inv = Matrix4f::Identity();
const int k_max = 1000, n_max = 1000;
const int rays_per_pixel = 4;

}

int main(int argc, char* argv[]) {
  assert(argc > 1);
  string obj_name = argv[1];
  string out_name = RemoveFileExtension(obj_name) + ".ppm";

  Scene scene;
  OBJParser::LoadOBJ(obj_name, &scene);

  MonteCarlo monte_carlo(&scene);
  monte_carlo.SetParameters(w, h, MVP_inv, k_max, n_max, rays_per_pixel);
  monte_carlo();
  const float* image = monte_carlo.result();
  SaveRGBToPPM(image, 640, 480, out_name);

  return 0;
}
