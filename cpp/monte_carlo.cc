#include "monte_carlo.h"

#include "eigen.h"
#include "geometry.h"
#include "object.h"
#include "path_tracer.h"
#include "random.h"

#include <cassert>
#include <cmath>
#include <chrono>
#include <ratio>

#ifdef VERBOSE
#include <iomanip>
#include <iostream>
#endif

void MonteCarlo::operator()(int n_max) {
  assert(are_params_set_ && "Should set up parameters frist!");
#ifdef VERBOSE
  int setw = std::ceil(std::log10(n_max + 1));
#endif

#ifdef VERBOSE
  auto start = std::chrono::system_clock::now();
#endif

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic) num_threads(16)
#endif
  for (int n = 0; n < n_max; ++n) {
#ifdef VERBOSE
    auto start = std::chrono::system_clock::now();
#endif

    for (int p = 0; p < cam_.w * cam_.h; ++p) {
      int u = p % cam_.w;
      int v = p / cam_.w;
      Vector3f estimator = sample(u, v);
      result_[p * 3 + 0] += estimator.x();
      result_[p * 3 + 1] += estimator.y();
      result_[p * 3 + 2] += estimator.z();
    }

#ifdef VERBOSE
  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double, std::ratio<1, 1000>> milliseconds = end - start;
  std::cout << '\n'
            << "| <Iteration " << std::setw(setw) << n + 1 << "/" << n_max << ">"
            << " [==============] " << milliseconds.count() << "ms";
#endif
  }

#ifdef VERBOSE
  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double, std::ratio<1, 1000>> total_ms = end - start;
  std::cout << "\n|\n| Total time: " << std::setprecision(2) << std::fixed
            << total_ms.count() << "ms(" << total_ms.count() / 60000 << "min)\n"
            << std::flush;
#endif
}

Vector3f MonteCarlo::sample(float u, float v) {
  std::vector<PathTracer::Path> paths;
  Vector3f estimator;

  backtrace(u, v, &paths);
  propagate(paths, &estimator);

  return estimator;
}

void MonteCarlo::backtrace(float u, float v, std::vector<PathTracer::Path>* paths) {
  Vector3f view_dir = cam_.R * Vector3f((u - cam_.cx) / cam_.fx,
                                        (v - cam_.cy) / cam_.fy, 1);
  Vector4f view_point = Vector4f(cam_.t);
  view_point.w() = 1;

  PathTracer::Path path(Ray(view_point, view_dir), view_dir.Normalize(), 1, nullptr);

  float path_pdf = path.p;
  paths->push_back(path);
  for (int i = 0; i < k_max_; ++i) {
    PathTracer::Path next_path = tracer_(path.r);
    path = next_path;

    path_pdf *= next_path.p;
    if (path_pdf <= Utils::Epsilon())
      break;

    paths->push_back(next_path);
    if (next_path.mtl->Ka.r() > 0 ||
        next_path.mtl->Ka.g() > 0 ||
        next_path.mtl->Ka.b() > 0)
      break;
  }
}

void MonteCarlo::propagate(const std::vector<PathTracer::Path>& paths, Vector3f* estimator) {
  /* Prepares for the first light transport. */
  float pdf = 1;
  Vector3f Ii;
  Vector3f V, L, N, H;
  Vector3f Ia, Id, Is;

  for (auto rit = paths.crbegin(); rit != paths.crend() - 1; ++rit) {
    const Material* mtl = rit->mtl;

    /* opaque: BRDF of Blinn-Phong model */
    if (mtl->Tr == 0) {
      V = -(rit + 1)->r.dir;
      H = (-L + V).Normalize();
      N = rit->N * V < 0 ? -rit->N : rit->N;

      Ia = mtl->Ka;
      Id.SetZero();
      Is.SetZero();

      /* diffusion */
      float cos_phi = N * (-L);
      Id = Vector3f(Ii.r() * mtl->Kd.r(),
                    Ii.g() * mtl->Kd.g(),
                    Ii.b() * mtl->Kd.b()) * cos_phi;
      if (mtl->Ks.r() > 0 || mtl->Ks.g() > 0 || mtl->Ks.b() > 0) {
        /* specular */
        float cos_theta = H * N;
        Is = Vector3f(Ii.r() * mtl->Ks.r(),
                      Ii.g() * mtl->Ks.g(),
                      Ii.b() * mtl->Ks.b()) * pow(cos_theta, mtl->Ns);
      }

      Ii = Ia + Id + Is;
      L = V;
      pdf *= ((rit + 1)->p);
    }

    /* transparent: BRDF of Lambert model */
    else {
      V = -(rit + 1)->r.dir;
      N = rit->N;

      Ia = mtl->Ka;
      Id.SetZero();

      /* lambert */
      float cos_phi = N * (-L);
      if (cos_phi > 0) {  /* on entering a transparent object */
        Id = Vector3f(Ii.r() * mtl->Kd.r(),
                      Ii.g() * mtl->Kd.g(),
                      Ii.b() * mtl->Kd.b()) * cos_phi;
      } else {            /* on leaveing a transparent object */
        Id = mtl->Tr * Vector3f(Ii.r() * mtl->Kd.r(),
                                Ii.g() * mtl->Kd.g(),
                                Ii.b() * mtl->Kd.b()) * -cos_phi;
      }

      Ii = Ia + Id;
      L = V;
      pdf *= ((rit + 1)->p);
    }
  }

  *estimator = Ii / pdf;
}
