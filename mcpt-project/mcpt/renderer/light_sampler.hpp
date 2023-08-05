#pragma once

#include <functional>
#include <optional>
#include <vector>

#include <Eigen/Eigen>

#include "mcpt/common/geometry/types.hpp"
#include "mcpt/common/object/material.hpp"
#include "mcpt/common/object/mesh.hpp"
#include "mcpt/common/object/object.hpp"
#include "mcpt/common/random.hpp"

#include "mcpt/renderer/ray_caster.hpp"

namespace mcpt {

class LightSampler {
public:
  struct PathToLight {
    std::reference_wrapper<const Material> material;  // light source material

    Eigen::Vector3f point;   // intersection point
    Eigen::Vector3f normal;  // surface normal at the intersection

    Eigen::Vector3f hit_dir;
    double hit_pdf;
  };

  LightSampler(const Object& object, const BVHTree<float>& bvh_tree);

  std::optional<PathToLight> Run(const Eigen::Vector3f& incident_point,
                                 const Eigen::Vector3f& incident_normal);

private:
  struct mesh_light {
    std::reference_wrapper<const Mesh> mesh;
    ConvexPolygon<float> triangle;
    float area;
    float accum_area;
  };

  std::reference_wrapper<const Object> m_associated_object;
  std::vector<mesh_light> m_mesh_lights;

  RayCaster m_ray_caster;
  Uniform<float> m_uni_area;
  UniformTriangle<float> m_uni_tri;
};

}  // namespace mcpt
