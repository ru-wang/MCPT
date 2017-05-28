#ifndef MCPT_PATH_TRACER_H_
#define MCPT_PATH_TRACER_H_

#include "eigen.h"
#include "geometry.h"
#include "random.h"
#include "ray_caster.h"

class Scene;
struct Material;

class PathTracer {
 public:
  struct Path {
    Path(const Ray& r, const Vector3f& N, float p, const Material* mtl)
        : r(r), N(N), p(p), mtl(mtl) {}

    Ray r;       /* the ray of the path */
    Vector3f N;  /* the normal at the interface */
    float p;     /* the probability */
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
  Path operator()(const Ray& r, float Ni = 1);

 private:
  RayCaster caster_;
  CosineWeightedHemisphere random_;
  const Scene* const scene_;
};

#endif  /* MCPT_PATH_TRACER_H_ */
