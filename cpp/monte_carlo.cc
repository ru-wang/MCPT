#include "monte_carlo.h"

#include "eigen.h"
#include "geometry.h"
#include "object.h"
#include "path_tracer.h"
#include "random.h"

#include <cassert>
#include <cmath>

#ifdef VERBOSE
#include <iostream>
#endif

using namespace std;

void MonteCarlo::operator()() {
  assert(are_params_set_ && "Should set up parameters frist!");

  result_ = new float[w_ * h_ * 3]{0};
  for (int n = 0; n < n_max_; ++n) {
#ifdef VERBOSE
    std::cerr << "| <Iteration " << n + 1 << "> [=========";
#endif
    for (int k = 0; k < k_max_; ++k) {
      for (int u = 0; u < w_; ++u) {
        for (int v = 0; v < h_; ++v) {
          for (int i = 0; i < rays_per_pixel_; ++i) {
            Vector3f color = sample(u, v, k);
            int p = (w_ * v + u) * 3;
            result_[p + 0] += color.r();
            result_[p + 1] += color.g();
            result_[p + 2] += color.b();
          }
        }
      }
    }
#ifdef VERBOSE
    float progress = (n + 1.0) / n_max_ * 100;
    std::cerr << "========= " << progress << "%]\n";
#endif
  }
  for (int p = 0; p < w_ * h_ * 3; p += 3)
    result_[p] /= n_max_;
}

Vector3f MonteCarlo::sample(int u, int v, int k) {
  Point view_point = MVP_inv_ * Vector4f(u, v, 1, 1);
  Vector3f view_dir = Vector3f(MVP_inv_ * Vector4f(u, v, 1, 0));

  vector<PathTracer::Path> paths;
  Ray r(view_point, view_dir);
  float Ni = 1;
  for (int i = 0; i < k; ++i) {
    PathTracer::Path path = tracer_(r, Ni);
    if (path.p == 0)
      break;
    else
      paths.push_back(path);
    r = path.r;
    Ni = path.mtl.Ni;
  }

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
      N = rit->N;
      H = (L + V).Normalize();

      Ia = mtl.Ka;
      Id.SetZero();
      Is.SetZero();
      float cos_phi = N * L;
      float cos_theta = H * N;
      if (cos_phi > 0)
        Id = Vector3f(Ii.x() * mtl.Kd.x(),
                      Ii.y() * mtl.Kd.y(),
                      Ii.z() * mtl.Kd.z()) * cos_phi;
      if (cos_theta > 0)
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

