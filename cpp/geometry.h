#ifndef MCPT_GEOMETRY_H_
#define MCPT_GEOMETRY_H_

#include "eigen.h"

#include <cstddef>
#include <vector>

/*
 * Here we define several primitives in P3 space.
 * All primitives are defined by homogeneous coordinates.
 */

  /*-------------------------------- Points --------------------------------*/

/*
 * Points are defined by their homogeneous coordinates in P3 space.
 */
typedef Vector4f Point;

  /*-------------------------------- Lines ---------------------------------**
   *
   * "Lines are very awkward to represent in 3-space since a natural
   *  representation for an object with 4 degrees of freedom would be
   *  a homogeneous 5-vector."
   *                            --[Multiple View Geometry 3.2.2, p68]
   *
   *------------------------------------------------------------------------*/

/*
 * Here we use the Plücker matrices to represent lines.
 * Suppose A , B are two(non-coincident) space points. Then the line joining
 * these points is represented by the 4×4 matrix:
 *
 *                           L = ABᵀ - BAᵀ,
 *
 * where L has rank 2. Its 2-dimensional null-space is spanned by the pencil
 * of planes with the line as axis(in fact LW*ᵀ=0, with 0 a 4×2 null-matrix).
 *
 * Join and incidence properties are very nicely represented in this notation:
 *  (i) The plane defined by the join of the point X and line L is
 *
 *                              π = L*X
 *
 *      and L*X=0 if, and only if, X is on L.
 * (ii) The point defined by the intersection of the line L with the plane π is
 *
 *                              X = Lπ
 *
 *      and Lπ = 0 if, and only if, L is on π.
 * And a dual Plücker representation L* can be obtained directly from L by a
 * simple rewrite rule:
 *
 *                    l₁₂ : l₁₃ : l₁₄ : l₂₃ : l₄₂ : l₃₄
 *                 =  l*₃₄ : l*₄₂ : l*₂₃ : l*₁₄ : l*₁₃ : l*₁₂.
 *
 */
struct Line {
  Line(const Vector3f& v1, const Vector3f& v2) {
    A.x() = v1.x(); A.y() = v1.y(); A.z() = v1.z(); A.w() = 1;
    B.x() = v2.x(); B.y() = v2.y(); B.z() = v2.z(); B.w() = 1;
  }

  Line(const Point& x1, const Point& x2) : A(x1), B(x2) {}

  Vector4f A;
  Vector4f B;
};

/*
 * As for the rays, the degrees of freedom is 5, 4 for the lines and plus 1 for
 * the endpoint.
 *
 * We still use the same data structure with Line here, with A being the endpoint
 * for the ray.
 */
struct Ray : public Line {
  Ray(const Vector3f& start_v, const Vector3f& dir)
      : Line(start_v, start_v + dir), dir(dir.Normalize()) {}

  Ray(const Point& start_x, const Vector3f& dir)
      : Line(start_x, start_x + Vector4f(dir)), dir(dir.Normalize()) {}

  void NormalizeInPlace() {
    dir.NormalizeInPlace();
  }

  Vector3f dir;
};

/*
 * The segments have 6 degrees of freedom, 4 for the lines plus 2 for the two
 * endpoints.
 *
 * We still use the same data structure with Line here, with A, B being the two
 * endpoints for the segment.
 */
struct Segment : public Line {
  Segment(const Vector3f& v1, const Vector3f& v2);
  Segment(const Point& x1, const Point& x2);
};

  /*-------------------------------- Planes --------------------------------**
   *
   * "A plane has 3 degrees of freedom in 3-space. The homogeneous
   * representation of the plane is the 4-vector."
   *                            --[Multiple View Geometry 3.2.1, p66]
   *------------------------------------------------------------------------*/

/*
 * Planes are defined by the implicit equation representation.
 */
typedef Vector4f Plane;

/*
 * The convex polygon class.
 */
class Polygon : public Plane {
 public:
  Polygon() = default;

  Polygon(const Vector3f& v1, const Vector3f& v2, const Vector3f& v3) {
    Vector3f d1 = v2 - v1;
    Vector3f d2 = v3 - v1;
    Vector3f normal = ::Cross(d1, d2).Normalize();
    float d = v1 * normal;
    Plane::x() = normal.x();
    Plane::y() = normal.y();
    Plane::z() = normal.z();
    Plane::w() = -d;
    v_.push_back(Vector4f{v1.x(), v1.y(), v1.z(), 1});
    v_.push_back(Vector4f{v2.x(), v2.y(), v2.z(), 1});
    v_.push_back(Vector4f{v3.x(), v3.y(), v3.z(), 1});
  }

  Polygon(const Vector3f& v1, const Vector3f& v2, const Vector3f& v3, const Vector3f& v4) {
    Vector3f d1 = v2 - v1;
    Vector3f d2 = v3 - v1;
    Vector3f normal = ::Cross(d1, d2);
    normal.NormalizeInPlace();
    Plane::x() = normal.x();
    Plane::y() = normal.y();
    Plane::z() = normal.z();
    Plane::w() = -v4 * normal;
    v_.push_back(Vector4f{v1.x(), v1.y(), v1.z(), 1});
    v_.push_back(Vector4f{v2.x(), v2.y(), v2.z(), 1});
    v_.push_back(Vector4f{v3.x(), v3.y(), v3.z(), 1});
    v_.push_back(Vector4f{v4.x(), v4.y(), v4.z(), 1});
  }

  std::vector<Point>& v() { return v_; }
  const std::vector<Point>& v() const { return v_; }

 private:
  std::vector<Point> v_;
};

  /*--------------------------------- AABB-- -------------------------------*/

/*
 * Axis-Aligned Bounding Box structrue.
 */
class AABB {
 public:
  AABB() {}

  AABB(const Vector4f& llb, const Vector4f& ruf)
      : llb_(llb), ruf_(ruf), diag_(ruf - llb) {
    ct_ = (llb_ + ruf_) / 2;
    ct_ /= ct_.w();
  }

  AABB(float left, float lower, float front, float right, float upper, float back)
      : llb_{left, lower, front, 1}, ruf_{right, upper, back, 1}, diag_(ruf_ - llb_) {
    ct_ = (llb_ + ruf_) / 2;
    ct_ /= ct_.w();
  }

  Vector4f& llb() { return llb_; }
  Vector4f& ruf() { return ruf_; }
  Vector3f& diag() { return diag_; }
  Vector4f& ct() { return ct_; }

  const Vector4f& llb() const { return llb_; }
  const Vector4f& ruf() const { return ruf_; }
  const Vector3f& diag() const { return diag_; }
  const Vector4f& ct() const { return ct_; }

  void Reset(const Vector4f& llb, const Vector4f& ruf) {
    llb_ = llb;
    ruf_ = ruf;
    diag_ = Vector3f(ruf - llb);
    ct_ = (llb_ + ruf_) / 2;
    ct_ /= ct_.w();
  }

  void Reset(float left, float lower, float front, float right, float upper, float back) {
    llb_ = Vector4f(left, lower, front, 1);
    ruf_ = Vector4f(right, upper, back, 1);
    diag_ = Vector3f(ruf_ - llb_);
    ct_ = (llb_ + ruf_) / 2;
    ct_ /= ct_.w();
  }

  bool Envelop(const Point& x) const {
    if (x.x() >= llb_.x() && x.x() <= ruf_.x() &&
        x.y() >= llb_.y() && x.y() <= ruf_.y() &&
        x.z() >= llb_.z() && x.z() <= ruf_.z())
      return true;
    else
      return false;
  }

 private:
  Vector4f llb_;   /* the left-lower-back vertex */
  Vector4f ruf_;   /* the right-upper-front vertex */
  Vector3f diag_;  /* the diagonal vector from the llb corner to the ruf corner */

  Vector4f ct_;    /* the center of the bounding box */
};

#endif  /* MCPT_GEOMETRY_H_ */
