#ifndef MCPT_MONTE_CARLO_H_
#define MCPT_MONTE_CARLO_H_

#include "eigen.h"
#include "path_tracer.h"

class Scene;

  /*
   * Monte Carlo integrator class.
   *
   * Use BRDF(Bidirectional reflection distribution function) of Blinn-Phong model
   * and BTDF(Bidirectional transmittance distribution function) of Lambert model.
   */
class MonteCarlo {
 public:
  MonteCarlo(const Scene* const scene)
      : are_params_set_(false), result_(nullptr), tracer_(scene), scene_(scene) {}

  ~MonteCarlo() { if (result_) delete result_, result_ = nullptr; }

  const float* result() const { return result_; }

  void SetParameters(int w, int h,
                     float fx, float fy, float cx, float cy,
                     const Vector3f& t,
                     const Matrix3f& R,
                     int k_max, int n_max, int rays_per_pixel) {
    cam_.w = w; cam_.h = h;
    cam_.fx = fx; cam_.fy = fy; cam_.cx = cx; cam_.cy = cy;
    cam_.t = t; cam_.R = R;
    k_max_ = k_max; n_max_ = n_max;
    rays_per_pixel_ = rays_per_pixel;
    are_params_set_ = true;
  }

  void operator()();

 private:
  /*
   * Samples a path with k points and returns the color at the pixel (u, v).
   */
  Vector3f sample(int u, int v, int k);

  /*
   * Parameters for the virtual camera.
   */
  struct {
    int w, h;
    float fx, fy, cx, cy;
    Vector3f t;
    Matrix3f R;
  } cam_;

  /*
   * Parameters used by the Monte Carlo integrator.
   */
  int k_max_;           /* maximum path length for path tracer */
  int n_max_;           /* maximum sampling times for Monte Carlo integrator */
  int rays_per_pixel_;  /* eye rays per pixel */

  bool are_params_set_;
  float* result_;

  PathTracer tracer_;
  const Scene* const scene_;
};

#endif  /* MCPT_MONTE_CARLO_H_ */
