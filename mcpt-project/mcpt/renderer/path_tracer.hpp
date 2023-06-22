#pragma once

#include <functional>
#include <optional>

#include <Eigen/Eigen>

#include "mcpt/common/geometry/types.hpp"
#include "mcpt/common/object/material.hpp"
#include "mcpt/common/object/object.hpp"
#include "mcpt/common/random.hpp"

#include "mcpt/renderer/ray_caster.hpp"

namespace mcpt {

class PathTracer {
public:
  struct ReversePath {
    std::reference_wrapper<const Material> target_material;  // surface material at the intersection
    Eigen::Vector3f point;                                   // intersection point
    Eigen::Vector3f normal;                                  // surface normal at the intersection

    Eigen::Vector3f exit_dir;
    double exit_pdf;
  };

  explicit PathTracer(const Object& object) : m_associated_object(object), m_ray_caster(object) {}

  // return the exit path at the intersection of the incident ray and the surface
  std::optional<ReversePath> Run(const Ray<float>& incident_ray);

private:
  struct sample {
    Eigen::Vector3f direction;
    Eigen::Vector3f normal;
    double pdf;
  };

  sample NextDirection(const Eigen::Vector3f& incident,
                       const Eigen::Vector3f& normal,
                       const Material& material);

  sample SampleDirection(const Eigen::Vector3f& normal, float alpha);

private:
  std::reference_wrapper<const Object> m_associated_object;

  RayCaster m_ray_caster;
  UniformHemisphere<float> m_cos_hemi;
  CosPowHemisphere<float> m_cos_pow_hemi;
};

}  // namespace mcpt
