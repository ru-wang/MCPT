#include "object.h"

#include "brdf.h"
#include "eigen.h"
#include "geometry.h"
#include "scene.h"

Object::BVH::~BVH() {
  if (l_child) delete l_child;
  if (r_child) delete r_child;
}

Object::~Object() {
  for (auto* mesh : mesh_list_)
    delete mesh;
  if (bvh_root_)
    delete bvh_root_;
}

void Object::AddMaterial(const std::string& mtl_name) {
  material_table_.push_back(std::make_pair(mtl_name, mesh_list_.size()));
}

std::string Object::GetMaterialNameByMeshID(size_t mesh_id) const {
  std::string mtl_name = material_table_.front().first;
  for (auto it = material_table_.cbegin() + 1; it != material_table_.cend(); ++it) {
    size_t start_id = it->second;
    if (mesh_id < start_id)
      return mtl_name;
    else
      mtl_name = it->first;
  }
  return mtl_name;
}

const Material& Object::GetMaterialByMeshID(size_t mesh_id) const {
  return scene_->materials()[GetMaterialNameByMeshID(mesh_id)];
}

Polygon Object::GetPolygonByMeshID(size_t mesh_id) const {
  const Mesh* mesh = mesh_list_[mesh_id];
  if (mesh->v_num() == 3) {
    const TriMesh* tri = dynamic_cast<const TriMesh*>(mesh);
    const Vector3f& v1 = scene_->v()[tri->v_id[0]];
    const Vector3f& v2 = scene_->v()[tri->v_id[1]];
    const Vector3f& v3 = scene_->v()[tri->v_id[2]];
    return Polygon(v1, v2, v3);
  } else if (mesh->v_num() == 4) {
    const RectMesh* rect = dynamic_cast<const RectMesh*>(mesh);
    const Vector3f& v1 = scene_->v()[rect->v_id[0]];
    const Vector3f& v2 = scene_->v()[rect->v_id[1]];
    const Vector3f& v3 = scene_->v()[rect->v_id[2]];
    const Vector3f& v4 = scene_->v()[rect->v_id[3]];
    return Polygon(v1, v2, v3, v4);
  } else {
    return Polygon();
  }
}

void Object::ConstructBVH() {
  /* constructs the root node */
  AABB obj_aabb;
  std::vector<BVH*> bvh_leaves;
  for (size_t i = 1; i < mesh_list_.size(); ++i) {
    AABB aabb;
    const Mesh* mesh = mesh_list_[i];
    if (mesh->v_num() == 3) {
      const TriMesh* tri = dynamic_cast<const TriMesh*>(mesh);
      for (size_t i = 0; i < mesh->v_num(); ++i) {
        const Vector3f& v = scene_->v()[tri->v_id[i]];
        aabb.llb().x() = v.x() < aabb.llb().x() ? v.x() : aabb.llb().x();
        aabb.llb().y() = v.y() < aabb.llb().y() ? v.y() : aabb.llb().y();
        aabb.llb().z() = v.z() < aabb.llb().z() ? v.z() : aabb.llb().z();
        aabb.ruf().x() = v.x() > aabb.ruf().x() ? v.x() : aabb.ruf().x();
        aabb.ruf().y() = v.y() > aabb.ruf().y() ? v.y() : aabb.ruf().y();
        aabb.ruf().z() = v.z() > aabb.ruf().z() ? v.z() : aabb.ruf().z();
      }
    } else if (mesh->v_num() == 4) {
      const RectMesh* rect = dynamic_cast<const RectMesh*>(mesh);
      for (size_t i = 0; i < mesh->v_num(); ++i) {
        const Vector3f& v = scene_->v()[rect->v_id[i]];
        aabb.llb().x() = v.x() < aabb.llb().x() ? v.x() : aabb.llb().x();
        aabb.llb().y() = v.y() < aabb.llb().y() ? v.y() : aabb.llb().y();
        aabb.llb().z() = v.z() < aabb.llb().z() ? v.z() : aabb.llb().z();
        aabb.ruf().x() = v.x() > aabb.ruf().x() ? v.x() : aabb.ruf().x();
        aabb.ruf().y() = v.y() > aabb.ruf().y() ? v.y() : aabb.ruf().y();
        aabb.ruf().z() = v.z() > aabb.ruf().z() ? v.z() : aabb.ruf().z();
      }
    }
    bvh_leaves.push_back(new BVH(aabb, i));
    obj_aabb.llb().x() = aabb.llb().x() < obj_aabb.llb().x() ? aabb.llb().x() : obj_aabb.llb().x();
    obj_aabb.llb().y() = aabb.llb().y() < obj_aabb.llb().y() ? aabb.llb().y() : obj_aabb.llb().y();
    obj_aabb.llb().z() = aabb.llb().z() < obj_aabb.llb().z() ? aabb.llb().z() : obj_aabb.llb().z();
    obj_aabb.ruf().x() = aabb.ruf().x() > obj_aabb.ruf().x() ? aabb.ruf().x() : obj_aabb.ruf().x();
    obj_aabb.ruf().y() = aabb.ruf().y() > obj_aabb.ruf().y() ? aabb.ruf().y() : obj_aabb.ruf().y();
    obj_aabb.ruf().z() = aabb.ruf().z() > obj_aabb.ruf().z() ? aabb.ruf().z() : obj_aabb.ruf().z();
  }

  /* constructs the subnodes */
  SpanBVHTree(bvh_leaves, &bvh_root_);
  if (bvh_root_->l_child)
    UpdateAABB(bvh_root_->l_child->aabb, &bvh_root_);
  if (bvh_root_->r_child)
    UpdateAABB(bvh_root_->r_child->aabb, &bvh_root_);
}

void Object::UpdateAABB(const AABB& new_aabb, BVH** const bvh) {
  AABB& old_aabb = (*bvh)->aabb;
  old_aabb.llb().x() = new_aabb.llb().x() < old_aabb.llb().x() ? new_aabb.llb().x() : old_aabb.llb().x();
  old_aabb.llb().y() = new_aabb.llb().y() < old_aabb.llb().y() ? new_aabb.llb().y() : old_aabb.llb().y();
  old_aabb.llb().z() = new_aabb.llb().z() < old_aabb.llb().z() ? new_aabb.llb().z() : old_aabb.llb().z();
  old_aabb.ruf().x() = new_aabb.ruf().x() > old_aabb.ruf().x() ? new_aabb.ruf().x() : old_aabb.ruf().x();
  old_aabb.ruf().y() = new_aabb.ruf().y() > old_aabb.ruf().y() ? new_aabb.ruf().y() : old_aabb.ruf().y();
  old_aabb.ruf().z() = new_aabb.ruf().z() > old_aabb.ruf().z() ? new_aabb.ruf().z() : old_aabb.ruf().z();
}

void Object::SpanBVHTree(std::vector<BVH*>& bvh_list, BVH** const bvh_parent) {
  if (bvh_list.size() == 1) {
    delete *bvh_parent;
    *bvh_parent = bvh_list.back();
    bvh_list.pop_back();
    return;
  }

  BVH* left_bvh = new BVH;
  BVH* right_bvh = new BVH;

  /* defines a hyper-plane in P3 space,
   * splitting the space into two partitions */
  const AABB& aabb_parent = (*bvh_parent)->aabb;
  Vector4f hyper_plane = Vector4f::Zero();
  if (aabb_parent.diag().x() >= aabb_parent.diag().y() &&
      aabb_parent.diag().x() >= aabb_parent.diag().z()) {
    hyper_plane.x() = 1;
    hyper_plane.w() = -aabb_parent.ct().x();
    Vector4f new_llb = aabb_parent.llb() + Vector4f{aabb_parent.diag().x() / 2};
    Vector4f new_ruf = aabb_parent.ruf() - Vector4f{aabb_parent.diag().x() / 2};
    left_bvh->aabb.Reset(aabb_parent.llb(), new_ruf);
    right_bvh->aabb.Reset(new_llb, aabb_parent.ruf());
  } else if (aabb_parent.diag().y() >= aabb_parent.diag().x() &&
             aabb_parent.diag().y() >= aabb_parent.diag().z()) {
    hyper_plane.y() = 1;
    hyper_plane.w() = -aabb_parent.ct().y();
    Vector4f new_llb = aabb_parent.llb() + Vector4f{0, aabb_parent.diag().y() / 2};
    Vector4f new_ruf = aabb_parent.ruf() - Vector4f{0, aabb_parent.diag().y() / 2};
    left_bvh->aabb.Reset(aabb_parent.llb(), new_ruf);
    right_bvh->aabb.Reset(new_llb, aabb_parent.ruf());
  } else {
    hyper_plane.z() = 1;
    hyper_plane.w() = -aabb_parent.ct().z();
    Vector4f new_llb = aabb_parent.llb() + Vector4f{0, 0, aabb_parent.diag().z() / 2};
    Vector4f new_ruf = aabb_parent.ruf() - Vector4f{0, 0, aabb_parent.diag().z() / 2};
    left_bvh->aabb.Reset(aabb_parent.llb(), new_ruf);
    right_bvh->aabb.Reset(new_llb, aabb_parent.ruf());
  }

  std::vector<BVH*> left_bvh_list;
  std::vector<BVH*> right_bvh_list;
  for (size_t i = 0; i < bvh_list.size(); ++i) {
    BVH* bvh = bvh_list.back();
    bvh_list.pop_back();
    if (bvh->aabb.ct() * hyper_plane + hyper_plane.w() <= 0)
      left_bvh_list.push_back(bvh);
    else
      right_bvh_list.push_back(bvh);
  }

  if (left_bvh_list.size()) {
    (*bvh_parent)->l_child = left_bvh;
    SpanBVHTree(left_bvh_list, &left_bvh);
    if (right_bvh_list.size()) {
      SpanBVHTree(right_bvh_list, &right_bvh);
      (*bvh_parent)->r_child = right_bvh;
    } else {
      delete right_bvh;
    }
  } else {
    SpanBVHTree(right_bvh_list, &right_bvh);
    (*bvh_parent)->l_child = right_bvh;
    delete left_bvh;
  }

  /* updates AABB */
  if ((*bvh_parent)->l_child)
    UpdateAABB((*bvh_parent)->l_child->aabb, bvh_parent);
  if ((*bvh_parent)->r_child)
    UpdateAABB((*bvh_parent)->r_child->aabb, bvh_parent);
}
