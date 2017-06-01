#include "monte_carlo.h"
#include "obj_parser.h"
#include "scene.h"
#include "utils.h"

#include <cassert>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

#ifdef VERBOSE
#include <cstring>
#include <iomanip>
#include <iostream>
#endif

using namespace std;

namespace {

/* camera parameters */
const int w = 640, h = 480;
const double fx = 480;
const double fy = 480;
const double cx = w / 2.0;
const double cy = h / 2.0;
//const Vector3f t = Vector3f(0, 5, 16);
//const Vector3f t = Vector3f(2, 3, 20);
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
  std::cout << std::setfill('0');
  for (int i = 3; i < argc - 1; ++i)
    std::cout << std::setw(std::strlen(argv[argc - 1]))
              << argv[i] << ",";
  std::cout << std::setw(std::strlen(argv[argc - 1]))
            << argv[argc - 1] << "}.ppm\n";
  std::cout << std::setfill(' ');
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

  Vector3f t;
  if (raw_name.back() == '1')
    t = Vector3f(0, 5, 16);
  else if (raw_name.back() == '2')
    t = Vector3f(2, 3, 20);
  else
    t = Vector3f(0, 5, 16);

  int k_max = stoi(argv[2]);
  int n_max = 0, last_n_max = 0;
  MonteCarlo monte_carlo(&scene);
  monte_carlo.SetParameters(w, h, fx, fy, cx, cy, t, R, k_max);
  int setwidth = std::strlen(argv[argc - 1]);
  for (int i = 3; i < argc; last_n_max = n_max, ++i) {
    n_max = stoi(argv[i]);
    monte_carlo(n_max - last_n_max);
    const double* image = monte_carlo.result();

    stringstream ss;
    ss << raw_name << "_" << setfill('0') << setw(setwidth) << n_max << ".ppm";
    string out_name = ss.str();
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
