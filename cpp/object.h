#ifndef MCPT_OBJECT_H_
#define MCPT_OBJECT_H_

#include "brdf.h"
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
   *         +-------------------------+.
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

  Object() : smooth_(0), mesh_list_(1), bvh_root_(nullptr) {}
  ~Object();

  int smooth() const { return smooth_; }
  const std::vector<Mesh*>& mesh_list() const { return mesh_list_; }
  const BVH* bvh_root() const { return bvh_root_; }

  void set_smooth(int smooth) { smooth_ = smooth; }
  std::vector<Mesh*>& mesh_list() { return mesh_list_; }
  BVH* bvh_root() { return bvh_root_; }

  void AddMaterial(const std::string& mtl_name);
  std::string GetMaterialNameByMeshID(size_t mesh_id) const;
  const Material& GetMaterialByMeshID(size_t mesh_id) const;
  Polygon GetPolygonByMeshID(size_t mesh_id) const;

  /*
   * Construct a BVH tree for itself.
   */
  void ConstructBVH();

 private:
  void UpdateAABB(const AABB& new_aabb, BVH** const bvh);
  void SpanBVHTree(std::vector<BVH*>& bvh_list, BVH** const bvh_parent);

  int smooth_;
  std::string material_;

  /* mesh list for this object, index: [1, N] */
  std::vector<Mesh*> mesh_list_;

  /* stores material information for meshes */
  std::vector<std::pair<std::string, size_t>> material_table_;

  BVH* bvh_root_;
  Scene* scene_;
};

#endif  /* MCPT_OBJECT_H_ */
