#ifndef MCPT_GEOMETRY_H_
#define MCPT_GEOMETRY_H_

#include <string>
#include <unordered_map>
#include <vector>

struct Vector2f {
  Vector2f() : v{0} {}
  float& operator[](size_t i) { return v[i]; }
  union {
    struct { float x, y; };
    float v[2];
  };
};

struct Vector3f {
  Vector3f() : v{0} {}
  float& operator[](size_t i) { return v[i]; }
  union {
    struct { float x, y, z; };
    struct { float r, g, b; };
    float v[3];
  };
};

struct Vector3i {
  Vector3i() : v{0} {}
  int& operator[](size_t i) { return v[i]; }
  union {
    struct { int x, y, z; };
    struct { int r, g, b; };
    int v[3];
  };
};

struct Vector4i {
  Vector4i() : v{0} {}
  int& operator[](size_t i) { return v[i]; }
  union {
    struct { int x, y, z, w; };
    struct { int r, g, b, a; };
    int v[4];
  };
};

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

/*
 * Axis-Aligned Bounding Box structrue.
 */
struct AABB {
  AABB() = default;
  AABB(const Vector3f& llb, const Vector3f& ruf) : llb(llb), ruf(ruf) {}

  Vector3f llb;  /* The left-lower-back vertex */
  Vector3f ruf;  /* The right-upper-front vertex */
};

class Object {
 public:
  /*
   * Bounding Volume Hierarchies class.
   */
  class BVH {
  };

  Object() : v_stride_(0), smooth_(0) {}
  Object(int v_stride) : v_stride_(v_stride), smooth_(0) {}

  int v_stride() const { return v_stride_; }
  int smooth() const { return smooth_; }
  const std::string& material() const { return material_; }
  const std::vector<int>& v_id() const { return v_id_; }
  const std::vector<int>& vt_id() const { return vt_id_; }
  const std::vector<int>& vn_id() const { return vn_id_; }

  void set_v_stride(int v_stride) { v_stride_ = v_stride; }
  void set_smooth(int smooth) { smooth_ = smooth; }
  void set_material(const std::string& mtl_name) { material_ = mtl_name; }
  std::vector<int>& v_id() { return v_id_; }
  std::vector<int>& vt_id() { return vt_id_; }
  std::vector<int>& vn_id() { return vn_id_; }

 private:
  int v_stride_;
  int smooth_;
  std::string material_;

  std::vector<int> v_id_;
  std::vector<int> vt_id_;
  std::vector<int> vn_id_;

  BVH bvh_;
};

class Scene {
 public:
  Scene() : v_(1), vt_(1), vn_(1) {}

  std::unordered_map<std::string, Object>& objects() { return objects_; }
  std::unordered_map<std::string, Material>& materials() { return materials_; }

  std::vector<Vector3f>& v() { return v_;  }
  std::vector<Vector2f>& vt() { return vt_; }
  std::vector<Vector3f>& vn() { return vn_; }

  const std::unordered_map<std::string, Object>& objects() const { return objects_; }
  const std::unordered_map<std::string, Material>& materials() const { return materials_; }

  const std::vector<Vector3f>& v() const { return v_;  }
  const std::vector<Vector2f>& vt() const { return vt_; }
  const std::vector<Vector3f>& vn() const { return vn_; }

 private:
  std::unordered_map<std::string, Object> objects_;      /* all the objects in the scene */
  std::unordered_map<std::string, Material> materials_;  /* all types of materials used in the scene */

  std::vector<Vector3f> v_;   /* global vertex set, index: [1, N] */
  std::vector<Vector2f> vt_;  /* global vertex texture set, index: [1, N] */
  std::vector<Vector3f> vn_;  /* global vertex normal set, index: [1, N] */
};

#endif  /* MCPT_GEOMETRY_H_ */
