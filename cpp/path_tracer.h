#ifndef MCPT_PATH_TRACER_H_
#define MCPT_PATH_TRACER_H_

#include "eigen.h"
#include "geometry.h"
#include "random.h"
#include "ray_caster.h"
#include "utils.h"

#include <tuple>

class Scene;
struct Material;

class PathTracer {
 public:
  struct Path {
    Path(const Ray& r, const Vector3f& N, double p, const Material* mtl)
        : r(r), N(N), p(p), mtl(mtl) {}

    Ray r;       /* the ray of the path */
    Vector3f N;  /* the normal at the interface */
    double p;     /* the probability */
    const Material* mtl;
  };

  PathTracer(const Scene* const scene) : caster_(scene), scene_(scene) {}

  /*
   * Returns the new output path at the intersection of the input ray
   * and the surface in the scene, given the geometry relationship
   * as well as the material information.
   *
   * The optional Ni specifies the index of refraction of the space where
   * the light is traveling. The default value is 1, meaning the vacuo.
   */
  Path operator()(const Ray& r);

 private:
  Vector4f GenDirWithP(const Vector3f& L, const Vector3f& N, const Material& mtl);

  Vector4f GenCosinePowerDirWithP(const Vector3f& Nz, const Vector3f L, double Ns);
  Vector4f GenCosineDirWithP(const Vector3f& Nz);

  RayCaster caster_;
  CosinePowerWeightedHemisphere cpwh_;
  CosineWeightedHemisphere cwh_;
  const Scene* const scene_;
};

inline Vector4f PathTracer::GenCosinePowerDirWithP(const Vector3f& Nz, const Vector3f L, double Ns) {
  Vector3f aux;
  if (Utils::IsZero(Nz.x()) && Utils::IsZero(Nz.y()))
    aux = Vector3f(1, 0, 0);
  else
    aux = Vector3f(0, 0, 1);
  Vector3f Nx = (aux - aux * Nz * aux).Normalize();
  Vector3f Ny = Cross(Nz, Nx).Normalize();

  Vector3f V;
  double phi, theta, p;
  do {
    std::tie(phi, theta, p) = cpwh_(Ns);
    double sin_theta = std::sin(theta);
    Vector3f H = std::cos(phi) * sin_theta * Nx +
                 std::sin(phi) * sin_theta * Ny +
                 std::cos(theta) * Nz;
    H.NormalizeInPlace();
    V = L - 2 * (L * H) * H;
  } while (V * Nz <= Utils::Epsilon());

  Vector4f V_with_p(V.Normalize());
  V_with_p.w() = p;
  return V_with_p;
}

inline Vector4f PathTracer::GenCosineDirWithP(const Vector3f& Nz) {
  Vector3f aux;
  if (Utils::IsZero(Nz.x()) && Utils::IsZero(Nz.y()))
    aux = Vector3f(1, 0, 0);
  else
    aux = Vector3f(0, 0, 1);
  Vector3f Nx = (aux - aux * Nz * aux).Normalize();
  Vector3f Ny = Cross(Nz, Nx).Normalize();

  Vector3f V;
  double phi, theta, p;
  do {
    std::tie(phi, theta, p) = cwh_();
    double sin_theta = std::sin(theta);
    V = std::cos(phi) * sin_theta * Nx +
        std::sin(phi) * sin_theta * Ny +
        std::cos(theta) * Nz;
  } while (V * Nz <= Utils::Epsilon());

  Vector4f V_with_p(V.Normalize());
  V_with_p.w() = p;
  return V_with_p;
}

#endif  /* MCPT_PATH_TRACER_H_ */
