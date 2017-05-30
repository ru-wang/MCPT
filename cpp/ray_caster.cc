#include "ray_caster.h"

#include "eigen.h"
#include "geometry.h"
#include "intersection.h"
#include "object.h"
#include "scene.h"

#include <limits>
#include <queue>
#include <tuple>

using namespace std;

tuple<Point, Vector3f, string>
RayCaster::operator()(const Ray& r) const {
  Intersection intersectant;

  Point intersection = Point::Zero();
  float min_t = -1;

  const Mesh* mesh = nullptr;
  string mtl_name;

  /* computes intersections with all the objects, selects the closest one */
  for (const auto& entry : scene_->objects()) {
    const Object& obj = entry.second;
    tuple<Point, float, size_t> tp = cast(r, obj, intersectant);
    float t = get<1>(tp);
    if (not Utils::IsZero(get<0>(tp).w())) {
      if (t < min_t || min_t < 0) {
        intersection = get<0>(tp);
        min_t = t;

        mesh = &obj.GetMeshByMeshID(get<2>(tp));
        mtl_name = obj.GetMaterialNameByMeshID(get<2>(tp));
      }
    }
  }

  if (min_t > 0) {
    /* computes the normal vector */
    Vector3f normal = Vector3f::Zero();
    if (mesh->v_num() == 3) {
      const TriMesh* tri = dynamic_cast<const TriMesh*>(mesh);
      for (size_t i = 0; i < mesh->v_num(); ++i)
        normal += scene_->vn()[tri->vn_id[i]];
    } else if (mesh->v_num() == 4) {
      const RectMesh* rect = dynamic_cast<const RectMesh*>(mesh);
      for (size_t i = 0; i < mesh->v_num(); ++i)
        normal += scene_->vn()[rect->vn_id[i]];
    }
    normal.NormalizeInPlace();
    return make_tuple(intersection, normal, mtl_name);
  } else {
    Vector3f nan = Vector3f::All(numeric_limits<float>::quiet_NaN());
    return make_tuple(intersection, nan, mtl_name);
  }
}

tuple<Point, float, size_t>
RayCaster::cast(const Ray& r, const Object& obj, const Intersection& intersectant) const {
  Point intersection = Point::Zero();
  size_t mesh_id = 0;
  float min_t = -1;

  /* traverses the BVH tree using BFS */
  queue<const Object::BVH*> queue;
  queue.push(obj.bvh_root());
  while (not queue.empty()) {
    const Object::BVH* current_bvh = queue.front();
    queue.pop();

    const AABB& aabb = current_bvh->aabb;
    if (not intersectant(r, aabb))
      continue;

    if (current_bvh->nonleaf()) {
      queue.push(current_bvh->l_child);
      if (current_bvh->r_child)
        queue.push(current_bvh->r_child);
    } else if (current_bvh->leaf()) {
      Polygon f = obj.GetPolygonByMeshID(current_bvh->mesh_id);
      Point x = intersectant(r, f);
      float t = Vector3f(x - r.A) * r.dir;
      if (not Utils::IsZero(x.w())) {
        if (t < min_t || min_t < 0) {
          intersection = x;
          mesh_id = current_bvh->mesh_id;
          min_t = t;
        }
      }
    }
  }

  return make_tuple(intersection, min_t, mesh_id);
}
