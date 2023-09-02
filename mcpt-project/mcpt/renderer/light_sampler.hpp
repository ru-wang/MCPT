#pragma once

#include <functional>
#include <optional>
#include <vector>

#include <Eigen/Eigen>

#include "mcpt/common/geometry/bvh_tree.hpp"
#include "mcpt/common/geometry/types.hpp"
#include "mcpt/common/object/material.hpp"
#include "mcpt/common/object/mesh.hpp"
#include "mcpt/common/object/object.hpp"

#include "mcpt/renderer/ray_caster.hpp"

namespace mcpt {

class LightSampler {
public:
  struct PathToLight {
    std::reference_wrapper<const Material> material;  // light source material

    Eigen::Vector3f point;   // intersection point at the light surface
    Eigen::Vector3f normal;  // surface normal at the intersection

    Eigen::Vector3f hit_dir;
    double hit_pdf;
  };

  LightSampler(const Object& object, const BVHTree<float>& bvh_tree);

  std::optional<PathToLight> Run(const Eigen::Vector3f& start_point,
                                 const Eigen::Vector3f& start_normal);

private:
  struct TriangleLight {
    std::reference_wrapper<const Mesh> mesh;
    ConvexPolygon<float> triangle;
    float area;
    float accum_area;
  };

  struct sample {
    double pdf;
    Eigen::Vector3f point;
    Eigen::Vector3f direction;
  };

  void AddTriangleLights(const Mesh& light);

  sample HitDirection(const Eigen::Vector3f& point,
                      const Eigen::Vector3f& normal,
                      const TriangleLight& light);

  sample SamplePlaneLight(const Eigen::Vector3f& point,
                          const Eigen::Vector3f& normal,
                          const TriangleLight& light);

  sample SampleSphericalLight(const Eigen::Vector3f& point,
                              const Eigen::Vector3f& normal,
                              const TriangleLight& light,
                              const Eigen::Vector3f& A,
                              const Eigen::Vector3f& B,
                              const Eigen::Vector3f& C,
                              float area);

  bool IsVisible(const Ray<float>& hit_ray,
                 const Mesh& target,
                 const Eigen::Vector3f& normal,
                 float hit_dist) const;

private:
  std::reference_wrapper<const Object> m_associated_object;
  std::vector<TriangleLight> m_triangle_lights;
  RayCaster m_ray_caster;
};

}  // namespace mcpt
