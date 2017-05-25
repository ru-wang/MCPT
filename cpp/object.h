#ifndef MCPT_OBJECT_H_
#define MCPT_OBJECT_H_

#include "geometry.h"

#include <string>
#include <vector>
#include <utility>

class Scene;

struct Mesh {
  virtual ~Mesh() {}
  virtual size_t v_num() const = 0;
};

struct TriMesh : public Mesh {
  TriMesh() = default;

  TriMesh(const std::vector<int>& ids) {
    for (size_t i = 0; i < 3; ++i) {
      v_id[i] = ids[i * 3 + 0];
      vt_id[i] = ids[i * 3 + 1];
      vn_id[i] = ids[i * 3 + 2];
    }
  }

  virtual ~TriMesh() override {}

  virtual size_t v_num() const override { return 3; }
  Vector3i v_id;
  Vector3i vt_id;
  Vector3i vn_id;
};

struct RectMesh : public Mesh {
  RectMesh() = default;

  RectMesh(const std::vector<int>& ids) {
    for (size_t i = 0; i < 4; ++i) {
      v_id[i] = ids[i * 3 + 0];
      vt_id[i] = ids[i * 3 + 1];
      vn_id[i] = ids[i * 3 + 2];
    }
  }

  virtual ~RectMesh() override {}

  virtual size_t v_num() const override { return 4; }
  Vector4i v_id;
  Vector4i vt_id;
  Vector4i vn_id;
};

struct Material {
  Material() : illum(0), Ks(0), Ns(0), Ni(1), Tr(0) {}

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
   * The Ks statement specifies the specular reflectivity using RGB values.
   */
  Vector3f Ks;

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

class Object {
 public:
  /*
   * Bounding Volume Hierarchies class.
   *
   * Represents a node in the binary BVH tree.
   * There are three types of nodes with 0/1/2 nodes respectively:
   *
   *   1. A full non-leaf node looks like this:
   *
   *         +-------------------------+
   *         |        [Parent]         |
   *         |           /\            |
   *         |          /  \           |
   *         |         /    \          |
   *         | [Left BVH]  [Right BVH] |
   *         +-------------------------+.
   *
   *   2. A half non-leaf node looks like this:
   *
   *         +-------------------------+
   *         |        [Parent]         |
   *         |           /\            |
   *         |          /  \           |
   *         |         /    \          |
   *         | [Left BVH]  [Null]      |
   *         +-------------------------+.
   *
   *   3. A leaf node looks like this:
   *
   *         +-------------------------+
   *         |        [Parent]         |
   *         |           /\            |
   *         |          /  \           |
   *         |         /    \          |
   *         |     [Null]  [ ⬟ ]       |
   *         +-------------------------+,
   *
   *      where [ ⬟ ] represents the mesh enclosed by the AABB.
   *      So a tree node with its left subnode being a null pointer
   *      is a leaf node.
   *
   *   4. A node with its type undecided may look like this:
   *
   *         +-------------------------+
   *         |        [Parent]         |
   *         |           /\            |
   *         |          /  \           |
   *         |         /    \          |
   *         |     [Null]  [Null]      |
   *         +-------------------------+,
   *
   *      and this shall not appear in the BVH tree.
   *
   */
  struct BVH {
    /*
     * Creates a BVH tree node with its type undecided.
     */
    BVH() : l_child(nullptr), mesh_id(0) {}

    /*
     * Creates a non-leaf BVH tree node with 1/2 subnodes.
     */
    BVH(const AABB& aabb, BVH* l_child, BVH* r_child = nullptr)
        : aabb(aabb), l_child(l_child), r_child(r_child) {}

    /*
     * Creates a leaf BVH tree node.
     */
    BVH(const AABB& aabb, int mesh_id)
        : aabb(aabb), l_child(nullptr), mesh_id(mesh_id) {}

    ~BVH();

    bool nonleaf() const { return l_child; }
    bool leaf() const { return !l_child && mesh_id; }

    AABB aabb;
    BVH* l_child;
    union {
      BVH* r_child;
      size_t mesh_id;
    };
  };

  Object(const Scene* const scene) : smooth_(0), mesh_list_(1),
                                     bvh_root_(nullptr), scene_(scene) {}
  ~Object();

  int smooth() const { return smooth_; }
  const std::vector<Mesh*>& mesh_list() const { return mesh_list_; }
  const BVH* bvh_root() const { return bvh_root_; }

  void set_smooth(int smooth) { smooth_ = smooth; }
  std::vector<Mesh*>& mesh_list() { return mesh_list_; }
  BVH* bvh_root() { return bvh_root_; }

  void AddMaterial(const std::string& mtl_name);
  const std::string& GetMaterialNameByMeshID(size_t mesh_id) const;
  const Material& GetMaterialByMeshID(size_t mesh_id) const;
  const Mesh& GetMeshByMeshID(size_t mesh_id) const;
  Polygon GetPolygonByMeshID(size_t mesh_id) const;

  /*
   * Construct a BVH tree for itself.
   */
  void ConstructBVH();

 private:
  void UpdateAABB(const AABB& new_aabb, BVH** const bvh);
  void SpanBVHTree(std::vector<BVH*>& bvh_list, BVH** const bvh_parent);

  int smooth_;

  /* mesh list for this object, index: [1, N] */
  std::vector<Mesh*> mesh_list_;

  /* stores material information for meshes */
  std::vector<std::pair<std::string, size_t>> material_table_;

  BVH* bvh_root_;
  const Scene* const scene_;

  Object(const Object&) = delete;
  Object& operator=(const Object&) = delete;
};

#endif  /* MCPT_OBJECT_H_ */
