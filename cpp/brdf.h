#ifndef MCPT_BRDF_H_
#define MCPT_BRDF_H_

#include "eigen.h"

struct Material {
  Material() : illum(0), Ns(0), Ni(0), Tr(0) {}

  /*
   * Illumination type.
   *
   * Currently we only allow 4, meaning:
   *   - transparency: glass on
   *   - reflection: ray trace on
   */
  float illum;

  /*
   * The Kd statement specifies the diffuse reflectivity using RGB values.
   */
  Vector3f Kd;

  /*
   * The Ka statement specifies the ambient reflectivity using RGB values.
   */
  Vector3f Ka;

  /*
   * The Ns exponent specifies the specular exponent for the current material.
   * This defines the focus of the specular highlight.
   */
  float Ns;

  /*
   * The Ni optical density specifies the optical density for the surface.
   * This is also known as index of refraction.
   */
  float Ni;

  /*
   * The Tr transparency specifies the transparency value for the material.
   * Unlike real transparency, the result does not depend upon the thickness of the object.
   * A value of 0.0 is the default and means fully opaque.
   */
  float Tr;
};

#endif  /* MCPT_BRDF_H_ */
