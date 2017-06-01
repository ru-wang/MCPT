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

  ~MonteCarlo() { if (result_) delete [] result_, result_ = nullptr; }

  const double* result() const { return result_; }

  void SetParameters(int w, int h,
                     double fx, double fy, double cx, double cy,
                     const Vector3f& t, const Matrix3f& R,
                     int k_max) {
    cam_.w = w; cam_.h = h;
    cam_.fx = fx; cam_.fy = fy; cam_.cx = cx; cam_.cy = cy;
    cam_.t = t; cam_.R = R;
    k_max_ = k_max;
    result_ = new double[cam_.w * cam_.h * 3]{0};
    are_params_set_ = true;
  }

  void operator()(int n_max);

 private:
  /*
   * Samples a path with k points and returns the color at the pixel (u, v).
   */
  Vector3f sample(double u, double v);

  void backtrace(double u, double v, std::vector<PathTracer::Path>* paths);
  void propagate(const std::vector<PathTracer::Path>& paths, Vector3f* estimator);

  /*
   * Parameters for the virtual camera.
   */
  struct {
    int w, h;
    double fx, fy, cx, cy;
    Vector3f t;
    Matrix3f R;
  } cam_;

  int k_max_;
  bool are_params_set_;
  double* result_;

  PathTracer tracer_;
  const Scene* const scene_;
};

#endif  /* MCPT_MONTE_CARLO_H_ */
