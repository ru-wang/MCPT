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

const int k_max = 10, n_max = 1;
const int rays_per_pixel = 1;

/* camera parameters */
const int w = 50, h = 50;
const float fx = 30;
const float fy = 30;
const float cx = w / 2.0;
const float cy = h / 2.0;
const Vector3f t = Vector3f(0, 5, 20);
const Matrix3f R = Matrix3f(1,  0,  0,
                            0, -1,  0,
                            0,  0, -1);

}

int main(int argc, char* argv[]) {
  assert(argc > 1);

#ifdef VERBOSE
  std::cerr << "\n"
            << "+++++++++++++++++++++ Reports +++++++++++++++++++++\n";
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
            << "| Vertex:   " << setw(8) << scene.v().size() - 1 << "\n"
            << "| Texture:  " << setw(8) << scene.vt().size() - 1 <<"\n"
            << "| Normal:   " << setw(8) << scene.vn().size() - 1 << "\n";
#endif

#ifdef VERBOSE
  std::cerr << "+                    _________                    |\n"
            << "+                        -                        |\n";
#endif

  scene.Initialize();

  MonteCarlo monte_carlo(&scene);
  monte_carlo.SetParameters(w, h,
                            fx, fy, cx, cy,
                            t, R,
                            k_max, n_max, rays_per_pixel);

  monte_carlo();

  const float* image = monte_carlo.result();
  Utils::SaveRGBToPPM(image, w, h, out_name);

#ifdef VERBOSE
  std::cerr << "+                        -                        |\n"
            << "+                    -* End *-                    +\n\n";
#endif

  return 0;
}
