#pragma once

#include <Eigen/Eigen>

namespace mcpt {

// http://paulbourke.net/dataformats/mtl/
struct Material {
  enum TypeEnum { EM, TR, SPEC, DIFF };

  // illumination type
  //
  // currently we only allow 4, meaning
  // - transparency: glass on
  // - reflection: ray trace on
  unsigned int illum = 0;

  // Ke r g b
  //
  // The Ke statement specifies the emission using RGB values.
  //
  // "r g b" are the values for the red, green, and blue components of the color. The g and b
  // arguments are optional. If only r is specified, then g, and b are assumed to be equal to r. The
  // r g b values are normally in the range of 0.0 to infinity.
  Eigen::Vector3f Ke{Eigen::Vector3f::Zero()};

  // Kd r g b
  //
  // The Kd statement specifies the diffuse reflectivity using RGB values.
  //
  // "r g b" are the values for the red, green, and blue components of the atmosphere. The g and b
  // arguments are optional. If only r is specified, then g, and b are assumed to be equal to r. The
  // r g b values are normally in the range of 0.0 to 1.0. Values outside this range increase or
  // decrease the relectivity accordingly.
  Eigen::Vector3f Kd{Eigen::Vector3f::Zero()};

  // Ka r g b
  //
  // The Ka statement specifies the ambient reflectivity using RGB values.
  //
  // "r g b" are the values for the red, green, and blue components of the color. The g and b
  // arguments are optional. If only r is specified, then g, and b are assumed to be equal to r. The
  // r g b values are normally in the range of 0.0 to 1.0. Values outside this range increase or
  // decrease the relectivity accordingly.
  Eigen::Vector3f Ka{Eigen::Vector3f::Zero()};

  // Ks r g b
  //
  // The Ks statement specifies the specular reflectivity using RGB values.
  //
  // "r g b" are the values for the red, green, and blue components of the atmosphere. The g and b
  // arguments are optional. If only r is specified, then g, and b are assumed to be equal to r. The
  // r g b values are normally in the range of 0.0 to 1.0. Values outside this range increase or
  // decrease the relectivity accordingly.
  Eigen::Vector3f Ks{Eigen::Vector3f::Zero()};

  // Ns exponent
  //
  // Specifies the specular exponent for the current material. This defines the focus of the
  // specular highlight.
  //
  // "exponent" is the value for the specular exponent. A high exponent results in a tight,
  // concentrated highlight. Ns values normally range from 0 to 1000.
  float Ns = 0.0F;

  // Ni optical_density
  //
  // Specifies the optical density for the surface. This is also known as index of refraction.
  //
  // "optical_density" is the value for the optical density. The values can range from 0.001 to 10.
  // A value of 1.0 means that light does not bend as it passes through an object. Increasing the
  // optical_density increases the amount of bending. Glass has an index of refraction of about 1.5.
  // Values of less than 1.0 produce bizarre results and are not recommended.
  float Ni = 1.0F;

  // d factor
  //
  // Specifies the dissolve for the current material.
  //
  // "factor" is the amount this material dissolves into the background. A factor of 1.0 is fully
  // opaque. This is the default when a new material is created. A factor of 0.0 is fully dissolved
  // (completely transparent).
  //
  // Unlike a real transparent material, the dissolve does not depend upon material thickness nor
  // does it have any spectral character. Dissolve works on all illumination models.
  //
  // Tr alpha
  //
  // The quantities d and Tr are the opposites of each other, and specifying transparency or
  // nontransparency is simply a matter of user convenience.
  float Tr = 0.0F;

  static TypeEnum Type(const Material& mtl) {
    if ((mtl.Ke.array() != 0.0F).any())
      return EM;
    else if (mtl.Tr != 0.0F)
      return TR;
    else if ((mtl.Kd.array() == 0.0F).all() && (mtl.Ks.array() != 0.0F).any())
      return SPEC;
    else
      return DIFF;
  }

  static const Eigen::Vector3f& AsEmission(const Material& mtl) { return mtl.Ke; }
};

inline bool operator==(const Material& lhs, const Material& rhs) {
  return lhs.illum == rhs.illum &&
         lhs.Ke == rhs.Ke &&
         lhs.Kd == rhs.Kd &&
         lhs.Ka == rhs.Ka &&
         lhs.Ks == rhs.Ks &&
         lhs.Ns == rhs.Ns &&
         lhs.Ni == rhs.Ni &&
         lhs.Tr == rhs.Tr;
}

}  // namespace mcpt
