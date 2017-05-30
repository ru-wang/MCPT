#ifndef MCPT_INTERSECTION_H_
#define MCPT_INTERSECTION_H_

#include "eigen.h"
#include "geometry.h"
#include "utils.h"

class Intersection {
 public:
  bool operator()(const Ray& r, const AABB& aabb) const;
  Point operator()(const Line& l, const Plane& pi) const;
  Point operator()(const Ray& r, const Plane& pi) const;
  Point operator()(const Ray& r, const Polygon& f) const;
};

inline bool Intersection::operator()(const Ray& r, const AABB& aabb) const {
  if (Utils::IsZero(r.dir.x()) && Utils::IsZero(r.dir.y()) && Utils::IsZero(r.dir.z()))
    return false;

  if (aabb.Envelop(r.A))
    return true;

  const Point& llb = aabb.llb();
  const Point& ruf = aabb.ruf();

  /* ray parallels to the Z axis */
  if (Utils::IsZero(r.dir.x()) && Utils::IsZero(r.dir.y())) {
    if (r.A.x() >= llb.x() && r.A.x() <= ruf.x() &&
        r.A.y() >= llb.y() && r.A.y() <= ruf.y() &&
        ((r.A.z() <= llb.z() && r.dir.z() >= 0) ||
         (r.A.z() >= ruf.z() && r.dir.z() <= 0)))
      return true;
    return false;
  } else

  /* ray parallels to the Y axis */
  if (Utils::IsZero(r.dir.x()) && Utils::IsZero(r.dir.z())) {
    if (r.A.x() >= llb.x() && r.A.x() <= ruf.x() &&
        r.A.z() >= llb.z() && r.A.z() <= ruf.z() &&
        ((r.A.y() <= llb.y() && r.dir.y() >= 0) ||
         (r.A.y() >= ruf.y() && r.dir.y() <= 0)))
      return true;
    return false;
  } else

  /* ray parallels to the X axis */
  if (Utils::IsZero(r.dir.y()) && Utils::IsZero(r.dir.z())) {
    if (r.A.y() >= llb.y() && r.A.y() <= ruf.y() &&
        r.A.z() >= llb.z() && r.A.z() <= ruf.z() &&
        ((r.A.x() <= llb.x() && r.dir.x() >= 0) ||
         (r.A.x() >= ruf.x() && r.dir.x() <= 0)))
      return true;
    return false;
  }

  else {
    /* projects the AABB to the XY plane */
    bool xy_intersectant = false;
    if ((r.A.x() <= llb.x() && r.dir.x() <= 0) ||
        (r.A.x() >= ruf.x() && r.dir.x() >= 0) ||
        (r.A.y() <= llb.y() && r.dir.y() <= 0) ||
        (r.A.y() >= ruf.y() && r.dir.y() >= 0)) {
      xy_intersectant = false;
    } else {
      Vector3f l_r = Cross(Vector3f(r.A.x(), r.A.y(), 1),
                           Vector3f(r.B.x(), r.B.y(), 1));
      Vector3f l1(1, 0, -llb.x()), l2(1, 0, -ruf.x());
      Vector3f c1 = Cross(l_r, l1);
      Vector3f c2 = Cross(l_r, l2);
      float t1 = c1.y() / c1.z();
      float t2 = c2.y() / c2.z();
      if ((t1 >= llb.y() && t1 <= ruf.y()) ||
          (t2 >= llb.y() && t2 <= ruf.y())) {
        xy_intersectant = true;
      } else {
        Vector3f l3(0, 1, -llb.y()), l4(0, 1, -ruf.y());
        Vector3f c3 = Cross(l_r, l3);
        Vector3f c4 = Cross(l_r, l4);
        float t3 = c3.x() / c3.z();
        float t4 = c4.x() / c4.z();
        if ((t3 >= llb.x() && t3 <= ruf.x()) ||
            (t4 >= llb.x() && t4 <= ruf.x()))
          xy_intersectant = true;
      }
    }

    /* projects the AABB to the XZ plane */
    bool xz_intersectant = false;
    if ((r.A.x() <= llb.x() && r.dir.x() <= 0) ||
        (r.A.x() >= ruf.x() && r.dir.x() >= 0) ||
        (r.A.z() <= llb.z() && r.dir.z() <= 0) ||
        (r.A.z() >= ruf.z() && r.dir.z() >= 0)) {
      xz_intersectant = false;
    } else {
      Vector3f l_r = Cross(Vector3f(r.A.x(), r.A.z(), 1),
                           Vector3f(r.B.x(), r.B.z(), 1));
      Vector3f l1(1, 0, -llb.x()), l2(1, 0, -ruf.x());
      Vector3f c1 = Cross(l_r, l1);
      Vector3f c2 = Cross(l_r, l2);
      float t1 = c1.y() / c1.z();
      float t2 = c2.y() / c2.z();
      if ((t1 >= llb.z() && t1 <= ruf.z()) ||
          (t2 >= llb.z() && t2 <= ruf.z())) {
        xz_intersectant = true;
      } else {
        Vector3f l3(0, 1, -llb.z()), l4(0, 1, -ruf.z());
        Vector3f c3 = Cross(l_r, l3);
        Vector3f c4 = Cross(l_r, l4);
        float t3 = c3.x() / c3.z();
        float t4 = c4.x() / c4.z();
        if ((t3 >= llb.x() && t3 <= ruf.x()) ||
            (t4 >= llb.x() && t4 <= ruf.x()))
          xz_intersectant = true;
      }
    }

    /* projects the AABB to the YZ plane */
    bool yz_intersectant = false;
    if ((r.A.y() <= llb.y() && r.dir.y() <= 0) ||
        (r.A.y() >= ruf.y() && r.dir.y() >= 0) ||
        (r.A.z() <= llb.z() && r.dir.z() <= 0) ||
        (r.A.z() >= ruf.z() && r.dir.z() >= 0)) {
      yz_intersectant = false;
    } else {
      Vector3f l_r = Cross(Vector3f{r.A.y(), r.A.z(), 1},
                           Vector3f{r.B.y(), r.B.z(), 1});
      Vector3f l1{1, 0, -llb.y()}, l2{1, 0, -ruf.y()};
      Vector3f c1 = Cross(l_r, l1);
      Vector3f c2 = Cross(l_r, l2);
      float t1 = c1.y() / c1.z();
      float t2 = c2.y() / c2.z();
      if ((t1 >= llb.z() && t1 <= ruf.z()) ||
          (t2 >= llb.z() && t2 <= ruf.z())) {
        yz_intersectant = true;
      } else {
        Vector3f l3(0, 1, -llb.z()), l4(0, 1, -ruf.z());
        Vector3f c3 = Cross(l_r, l3);
        Vector3f c4 = Cross(l_r, l4);
        float t3 = c3.x() / c3.z();
        float t4 = c4.x() / c4.z();
        if ((t3 >= llb.y() && t3 <= ruf.y()) ||
            (t4 >= llb.y() && t4 <= ruf.y()))
          yz_intersectant = true;
      }
    }

    return xy_intersectant && xz_intersectant && yz_intersectant;
  }
}

inline Point Intersection::operator()(const Line& l, const Plane& pi) const {
  Point x = l.A * (l.B * pi) - l.B * (l.A * pi);
  if (Utils::IsZero(x.w()))
    return Point::Zero();
  else
    return x / x.w();
}

inline Point Intersection::operator()(const Ray& r, const Plane& pi) const {
  const Line& l = static_cast<const Line&>(r);
  Point x = (*this)(l, pi);
  if (Utils::IsZero(x.w()))
    return Point::Zero();
  else if (Vector3f(x - r.A) * r.dir < Utils::Epsilon())
    return Point::Zero();
  else
    return x;
}

inline Point Intersection::operator()(const Ray& r, const Polygon& f) const {
  const Plane& pi = static_cast<const Plane&>(f);
  Point x = (*this)(r, pi);

  if (Utils::IsZero(x.w()))
    return Point::Zero();

  for (size_t i = 0; i < f.v().size(); ++i) {
    size_t j = i + 1 < f.v().size() ? i + 1 : 0;
    size_t k = j + 1 < f.v().size() ? j + 1 : 0;
    Vector3f AB(f.v()[j] - f.v()[i]);
    Vector3f PA(f.v()[i] - x);
    Vector3f CA(f.v()[i] - f.v()[k]);
    if (Cross(PA, AB) * Cross(CA, AB) <= 0)
      return Point::Zero();
  }
  return x;
}

#endif  /* MCPT_INTERSECTION_H_ */
