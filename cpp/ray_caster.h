#ifndef MCPT_RAY_CASTER_H_
#define MCPT_RAY_CASTER_H_

#include "eigen.h"
#include "geometry.h"

#include <tuple>

class Intersection;
class Object;
class Scene;

class RayCaster {
 public:
  RayCaster(const Scene* const scene) : scene_(scene) {}

  /*
   * Returns the intersection and the normal at the intersection of the ray
   * and the objects in the scene, as well as the material name of the mesh.
   */
  std::tuple<Point, Vector3f, std::string> operator()(const Ray& r0) const;

 private:

  /*
   * Returns the intersection of the ray and an object, and its projection length
   * on the ray, as well as the material name of the mesh.
   */
  std::tuple<Point, double, size_t>
  cast(const Ray& r, const Object& obj, const Intersection& intersectant) const;

  const Scene* const scene_;
};

#endif  /* MCPT_RAY_CASTER_H_ */
