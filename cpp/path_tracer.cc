#include "path_tracer.h"

#include "geometry.h"
#include "scene.h"

#include <cmath>
#include <string>
#include <tuple>

#ifdef VERBOSE
#include <iomanip>
#include <iostream>
#endif

using namespace std;

PathTracer::Path PathTracer::operator()(const Ray& r, float Ni) {
  Point intersection;
  Vector3f normal;
  string mtl_name;
  tie(intersection, normal, mtl_name) = caster_(r);
  const auto& entry = scene_->materials().find(mtl_name);
  const Material* mtl = entry != scene_->materials().cend() ? &entry->second : nullptr;

  /* No intersection, returns an invalid path with -1 probability. */
  if (isnan(normal.x()) || isnan(normal.y()) || isnan(normal.z())) {
#ifdef VERBOSE
//    std::cout << r.A << " ==> " << r.dir << "\n";
#endif
    return PathTracer::Path(r, normal, -1, mtl);
  }

  const Vector3f& L = r.dir;
  const Vector3f& N = normal;
  float cos_omega = -(L * N);

  /* opaque */
  if (mtl->Tr == 0) {
    Vector3f Nz = cos_omega < 0 ? -N : N;
    Vector3f Nx = (L + N * cos_omega).Normalize();
    Vector3f Ny = Cross(Nz, Nx).Normalize();

    Vector3f V;
    float phi, theta, p;
    do {
      tie(phi, theta, p) = random_();
      float sin_theta = sin(theta);
      Vector3f H = cos(phi) * sin_theta * Nx +
                   sin(phi) * sin_theta * Ny +
                   cos(theta) * Nz;
      H.NormalizeInPlace();
      V = L - 2 * (L * H) * H;
    } while (V * Nz <= 0);

    return PathTracer::Path(Ray(intersection, V), N, p, mtl);
  }

  /* transparent */
  else {
    if (cos_omega > 0) {
      /* enters a transparent object */
      float sin_omega = sqrt(1 - cos_omega * cos_omega);
      float sin_theta = sin_omega * Ni / mtl->Ni;
      float cos_theta = sqrt(1 - sin_theta * sin_theta);

      Vector3f Npar = N;
      Vector3f Nper = (L + N * cos_omega).Normalize();
      Vector3f V = Nper * sin_theta - Npar * cos_theta;
      float p = 1;

      return PathTracer::Path(Ray(intersection, V), N, p, mtl);
    } else {
      /* leaves a transparent object */
      Vector3f Nz = N;
      Vector3f Nx = (L + N * cos_omega).Normalize();
      Vector3f Ny = Cross(Nz, Nx).Normalize();

      Vector3f V;
      float phi, theta, p;

      do {
        tie(phi, theta, p) = random_();
        float sin_theta = sin(theta);
        V = cos(phi) * sin_theta * Nx +
            sin(phi) * sin_theta * Ny +
            cos(theta) * Nz;
      } while (V * N <= 0);

      return PathTracer::Path(Ray(intersection, V), N, p, mtl);
    }
  }
}
