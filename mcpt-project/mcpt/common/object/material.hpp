#pragma once

#include <Eigen/Eigen>

namespace mcpt {

struct Material {
  // illumination type
  //
  // currently we only allow 4, meaning
  // - transparency: glass on
  // - reflection: ray trace on
  unsigned int illum = 0;

  // Kd statement specifies the diffuse reflectivity using RGB values
  Eigen::Vector3f Kd{Eigen::Vector3f::Zero()};

  // Ka statement specifies the ambient reflectivity using RGB values
  Eigen::Vector3f Ka{Eigen::Vector3f::Zero()};

  // Ks statement specifies the specular reflectivity using RGB values
  Eigen::Vector3f Ks{Eigen::Vector3f::Zero()};

  // Ns exponent specifies the specular exponent for the current material
  // defines the focus of the specular highlight
  // Ns values normally range from 0 to 1000
  float Ns = 0.0F;

  // Ni optical density specifies the optical density for the surface
  // also known as index of refraction
  float Ni = 0.0F;

  // Tr transparency specifies the transparency value for the material
  // unlike real transparency, the result does not depend upon the thickness of the object
  // 0 means fully opaque
  float Tr = 0.0F;

  friend bool operator==(const Material& lhs, const Material& rhs) {
    return lhs.illum == rhs.illum &&
           lhs.Kd == rhs.Kd &&
           lhs.Ka == rhs.Ka &&
           lhs.Ks == rhs.Ks &&
           lhs.Ns == rhs.Ns &&
           lhs.Ni == rhs.Ni &&
           lhs.Tr == rhs.Tr;
  }
};

}  // namespace mcpt
