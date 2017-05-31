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

/* camera parameters */
const int w = 500, h = 500;
const float fx = 500;
const float fy = 500;
const float cx = w / 2.0;
const float cy = h / 2.0;
const Vector3f t = Vector3f(0, 5, 15);
const Matrix3f R = Matrix3f(1,  0,  0,
                            0, -1,  0,
                            0,  0, -1);

}

int main(int argc, char* argv[]) {
  assert(argc > 3);

#ifdef VERBOSE
  std::cout << "\n"
            << "+++++++++++++++++++++ Reports +++++++++++++++++++++\n";
#endif

  string obj_name = argv[1];
  string raw_name = Utils::RemoveSuffix(obj_name);

#ifdef VERBOSE
  std::cout << "| OBJ file: " << obj_name << "\n";
#endif

  Scene scene;
  OBJParser::LoadOBJ(obj_name, &scene);

#ifdef VERBOSE
  std::cout << "| Out file: " << raw_name << "_{";
  for (int i = 3; i < argc - 1; ++i)
    std::cout << argv[i] << ",";
  std::cout << argv[argc - 1] << "}.ppm\n";
#endif

#ifdef VERBOSE
  std::cout << "+                    *********                    |\n"
            << "+                                                 |\n";
#endif

#ifdef VERBOSE
  std::cout << "| Object:   " << setw(8) << scene.objects().size() << "\n"
            << "| Material: " << setw(8) << scene.materials().size() << "\n"
            << "| Vertex:   " << setw(8) << scene.v().size() - 1 << "\n"
            << "| Texture:  " << setw(8) << scene.vt().size() - 1 <<"\n"
            << "| Normal:   " << setw(8) << scene.vn().size() - 1 << "\n";
#endif

#ifdef VERBOSE
  std::cout << "+                    *********                    |\n"
            << "+                                                 |\n";
#endif

  scene.Initialize();

#ifdef VERBOSE
  std::cout << "+                    *********                    |\n"
            << "+                                                 |\n";
#endif

  int k_max = stoi(argv[2]);
  int n_max = 0, last_n_max = 0;
  MonteCarlo monte_carlo(&scene);
  monte_carlo.SetParameters(w, h, fx, fy, cx, cy, t, R, k_max);
  for (int i = 3; i < argc; last_n_max = n_max, ++i) {
    n_max = stoi(argv[i]);
    monte_carlo(n_max - last_n_max);
    const float* image = monte_carlo.result();

    string out_name = raw_name + "_" + string(argv[i]) + ".ppm";
    Utils::SaveRGBToPPM(image, n_max, w, h, out_name);

#ifdef VERBOSE
    std::cout << "+                                                 |\n"
              << "| Result written to: " << out_name << "\n"
              << "+                                                 |\n";
#endif
  }

#ifdef VERBOSE
  std::cout << "+                                                 |\n"
            << "+                  -*** End ***-                  +\n\n";
#endif

  return 0;
}
