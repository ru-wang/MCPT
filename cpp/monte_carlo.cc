#include "monte_carlo.h"

#include "eigen.h"
#include "geometry.h"
#include "object.h"
#include "path_tracer.h"
#include "random.h"

#include <cassert>
#include <cmath>

#ifdef VERBOSE
#include <iomanip>
#include <iostream>
#endif

using namespace std;

void MonteCarlo::operator()() {
  assert(are_params_set_ && "Should set up parameters frist!");

  result_ = new float[cam_.w * cam_.h * 3]{0};
  for (int n = 0; n < n_max_; ++n) {
#ifdef VERBOSE
    std::cerr << "| <Iteration " << n + 1 << "> [=========\n";
#endif
    for (int k = 1; k <= k_max_; ++k) {
      for (int v = 0; v < cam_.h; ++v) {
        for (int u = 0; u < cam_.w; ++u) {
//      for (int v = 0; v < 1; ++v) {
//        for (int u = 0; u < 1; ++u) {
          for (int i = 0; i < rays_per_pixel_; ++i) {
            Vector3f color = sample(u, v, k);
            int p = (cam_.w * v + u) * 3;
            result_[p + 0] += color.r();
            result_[p + 1] += color.g();
            result_[p + 2] += color.b();
          }
        }
      }
    }
#ifdef VERBOSE
    float progress = (n + 1.0) / n_max_ * 100;
    std::cerr << std::setprecision(2) << std::fixed << "========= " << progress << "%]\n";
#endif
  }
  for (int p = 0; p < cam_.w * cam_.h * 3; p += 3)
    result_[p] /= n_max_;
}

Vector3f MonteCarlo::sample(int u, int v, int k) {
  Vector3f view_dir = cam_.R * Vector3f((u - cam_.cx) / cam_.fx,
                                        (v - cam_.cy) / cam_.fy, 1);
  Vector4f view_point = Vector4f(cam_.t);
  view_point.w() = 1;
  Ray r(view_point, view_dir);

  vector<PathTracer::Path> paths;
  float Ni = 1;
  for (int i = 0; i < k; ++i) {
    PathTracer::Path path = tracer_(r, Ni);
    if (path.p < 0)
      break;
    else
      paths.push_back(path);
    r = path.r;
    Ni = path.mtl.Ni;
  }

#ifdef VERBOSE
  std::cout << "[" << setprecision(2)
//            << setw(5) << view_point.x() << " "
//            << setw(5) << view_point.y() << " "
//            << setw(5) << view_point.z() << " "
//            << setw(5) << view_point.w() << "] ==> ["
//            << setw(5) << view_dir.x() << " "
//            << setw(5) << view_dir.y() << " "
//            << setw(5) << view_dir.z() << "] "
            << "Path length: " << paths.size() << "]\n";
#endif

  /* Prepares for the first light transport. */
  float probability = 1;
  Vector3f Ii;
  Vector3f Ia, Id, Is;
  Vector3f V, L, N, H;

  for (auto rit = paths.crbegin(); rit != paths.crend(); ++rit) {
    const Material& mtl = rit->mtl;

    /* opaque: BRDF of Blinn-Phong model */
    if (mtl.Tr == 0) {
      V = -rit->r.dir;
      H = (-L + V).Normalize();
      N = rit->N * H > 0 ? rit->N : -rit->N;

      Ia = mtl.Ka;
      Id.SetZero();
      Is.SetZero();
      float cos_phi = N * L;
      float cos_theta = H * N;
      Id = Vector3f(Ii.x() * mtl.Kd.x(),
                    Ii.y() * mtl.Kd.y(),
                    Ii.z() * mtl.Kd.z()) * cos_phi;
      Is = Vector3f(Ii.x() * mtl.Ks.x(),
                    Ii.y() * mtl.Ks.y(),
                    Ii.z() * mtl.Ks.z()) * pow(cos_theta, mtl.Ns);
      Ii = Ia + Id + Is;

      L = V;
      probability *= rit->p;
    }

    /* transparent: BTDF of Lambertian model */
    else {
      V = -rit->r.dir;
      N = rit->N;

      Ia = mtl.Ka;
      Id.SetZero();
      float cos_phi = N * (-L);
      if (cos_phi > 0)
        Id = mtl.Tr * Vector3f(Ii.x() * mtl.Kd.x(),
                               Ii.y() * mtl.Kd.y(),
                               Ii.z() * mtl.Kd.z()) * cos_phi;
      else
        Id = mtl.Tr * Ii;
      Ii = Ia + Id;

      L = V;
      probability *= rit->p;
    }
  }

  return Ii;
}

