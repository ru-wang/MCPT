#pragma once

#include <Eigen/Eigen>

#include "mcpt/common/object/material.hpp"

namespace mcpt {

class BRDF {
public:
  virtual ~BRDF() noexcept = default;
  virtual Eigen::Vector3f Shade(const Material& p_mtl,
                                const Eigen::Vector3f& n,
                                const Eigen::Vector3f& wi_normalized, float wi_norm,
                                const Eigen::Vector3f& wo_normalized, float wo_norm) const = 0;
};

class BlinnPhongBRDF : public BRDF {
public:
  ~BlinnPhongBRDF() noexcept override = default;
  Eigen::Vector3f Shade(const Material& p_mtl,
                        const Eigen::Vector3f& n,
                        const Eigen::Vector3f& wi_normalized, float wi_norm,
                        const Eigen::Vector3f& wo_normalized, float wo_norm) const override;
};

}  // namespace mcpt
