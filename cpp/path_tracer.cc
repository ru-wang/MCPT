#include "path_tracer.h"

#include "geometry.h"
#include "scene.h"
#include "utils.h"

#include <cmath>
#include <string>
#include <tuple>

using namespace std;

PathTracer::Path PathTracer::operator()(const Ray& r) {
  Point intersection;
  Vector3f normal;
  string mtl_name;
  tie(intersection, normal, mtl_name) = caster_(r);
  const auto& entry = scene_->materials().find(mtl_name);
  const Material* mtl = entry != scene_->materials().cend() ? &entry->second : nullptr;

  /* No intersection, returns an invalid path with pdf=-1. */
  if (isnan(normal.x()) || isnan(normal.y()) || isnan(normal.z()))
    return PathTracer::Path(r, normal, -1, mtl);

  Vector4f V_with_p = GenDirWithP(r.dir, normal, *mtl);
  Vector3f V(V_with_p);
  float p = V_with_p.w();
  if (mtl->Tr == 0)
    return PathTracer::Path(Ray(intersection, V), normal, p, mtl);
  else
    return PathTracer::Path(Ray(intersection, V), V, p, mtl);
}

Vector4f PathTracer::GenDirWithP(const Vector3f& L, const Vector3f& N, const Material& mtl) {
  float cos_omega = -(L * N);

  /* opaque */
  if (mtl.Tr == 0) {
    Vector3f Nz = cos_omega < 0 ? -N : N;
    if (mtl.Ks.r() > 0 || mtl.Ks.g() > 0 || mtl.Ks.b() > 0)
      /* glossy */
      return GenCosinePowerDirWithP(Nz, L, mtl.Ns);
    else
      /* lambert */
      return GenCosineDirWithP(Nz);
  }

  /* transparent */
  else {
    float sin_omega = sqrt(1 - cos_omega * cos_omega);
    Vector3f Npar = N;
    Vector3f Nper = (L + N * cos_omega).Normalize();

    Vector3f R, V;
    float sin_theta, cos_theta;
    if (cos_omega > 0) {
      /* on entering a transparent object */
      sin_theta = sin_omega / mtl.Ni;
      cos_theta = sqrt(1 - sin_theta * sin_theta);
      R = (Nper * sin_theta - Npar * cos_theta).Normalize();
      return Vector4f(R.x(), R.y(), R.z(), 1);
    } else {
      /* on leaving a transparent object */
      sin_theta = sin_omega * mtl.Ni;
      if (sin_theta >= 1)
        /* total reflection */
        cos_theta = 0;
      /* refraction */
      else
        cos_theta = sqrt(1 - sin_theta * sin_theta);
      return GenCosineDirWithP(N);
    }
  }
}
