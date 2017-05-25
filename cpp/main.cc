#include "monte_carlo.h"
#include "obj_parser.h"
#include "scene.h"
#include "utils.h"

#include <cassert>
#include <fstream>
#include <string>

#ifdef VERBOSE
#include <iomanip>
#include <iostream>
#endif

using namespace std;

namespace {

const int w = 640, h = 480;
const Matrix4f MVP_inv = Matrix4f::Identity();
const int k_max = 1000, n_max = 1000;
const int rays_per_pixel = 4;

}

int main(int argc, char* argv[]) {
  assert(argc > 1);

#ifdef VERBOSE
  std::cerr << "+++++++++++++++++++++ Verbose +++++++++++++++++++++\n";
#endif

  string obj_name = argv[1];
  string out_name = Utils::RemoveSuffix(obj_name) + ".ppm";

#ifdef VERBOSE
  std::cerr << "| OBJ file: " << obj_name << "\n";
#endif

  Scene scene;
  OBJParser::LoadOBJ(obj_name, &scene);

#ifdef VERBOSE
  std::cerr << "| Out file: " << out_name << "\n";
#endif

#ifdef VERBOSE
  std::cerr << "+                    _________                    |\n"
            << "+                        -                        |\n";
#endif

#ifdef VERBOSE
  std::cerr << "| Object:   " << setw(8) << scene.objects().size() << "\n"
            << "| Material: " << setw(8) << scene.materials().size() << "\n"
            << "| Vertex:   " << setw(8) << scene.v().size() << "\n"
            << "| Texture:  " << setw(8) << scene.vt().size() << "\n"
            << "| Normal:   " << setw(8) << scene.vn().size() << "\n";
#endif

#ifdef VERBOSE
  std::cerr << "+                    _________                    |\n"
            << "+                        -                        |\n";
#endif

  scene.Initialize();

#ifdef VERBOSE
  std::cerr << "+                        -                        |\n"
            << "+                    -* End *-                    +\n\n";
#endif

  MonteCarlo monte_carlo(&scene);
  monte_carlo.SetParameters(w, h, MVP_inv, k_max, n_max, rays_per_pixel);
  monte_carlo();
  const float* image = monte_carlo.result();
  Utils::SaveRGBToPPM(image, 640, 480, out_name);

  return 0;
}
