#pragma once

#include <Eigen/Eigen>

#include "mcpt/common/object/material.hpp"

namespace mcpt {

class BxDF {
public:
  virtual ~BxDF() noexcept = default;
  virtual Eigen::Vector3f Shade(const Material& p_mtl,
                                const Eigen::Vector3f& n,
                                const Eigen::Vector3f& wi,
                                const Eigen::Vector3f& wo) const = 0;
};

class BlinnPhongBxDF : public BxDF {
public:
  ~BlinnPhongBxDF() noexcept override = default;
  Eigen::Vector3f Shade(const Material& p_mtl,
                        const Eigen::Vector3f& n,
                        const Eigen::Vector3f& wi,
                        const Eigen::Vector3f& wo) const override;
};

}  // namespace mcpt
