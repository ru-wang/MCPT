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

  const float* result() const { return result_; }

  void SetParameters(int w, int h,
                     float fx, float fy, float cx, float cy,
                     const Vector3f& t, const Matrix3f& R,
                     float pdf_epsilon) {
    cam_.w = w; cam_.h = h;
    cam_.fx = fx; cam_.fy = fy; cam_.cx = cx; cam_.cy = cy;
    cam_.t = t; cam_.R = R;
    pdf_epsilon_ = pdf_epsilon;
    result_ = new float[cam_.w * cam_.h * 3]{0};
    are_params_set_ = true;
  }

  void operator()(int n_max);

 private:
  /*
   * Samples a path with k points and returns the color at the pixel (u, v).
   */
  Vector3f sample(float u, float v);

  void backtrace(float u, float v, std::vector<PathTracer::Path>* paths);
  void propagate(const std::vector<PathTracer::Path>& paths, Vector3f* estimator);

  /*
   * Parameters for the virtual camera.
   */
  struct {
    int w, h;
    float fx, fy, cx, cy;
    Vector3f t;
    Matrix3f R;
  } cam_;

  float pdf_epsilon_;
  bool are_params_set_;
  float* result_;

  PathTracer tracer_;
  const Scene* const scene_;
};

#endif  /* MCPT_MONTE_CARLO_H_ */
