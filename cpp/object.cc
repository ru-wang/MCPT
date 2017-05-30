#include "object.h"

#include "eigen.h"
#include "geometry.h"
#include "scene.h"

#include <cassert>
#include <limits>

Object::BVH::~BVH() {
  if (nonleaf()) {
    delete l_child, l_child = nullptr;
    delete r_child, r_child = nullptr;
  }
}

Object::~Object() {
  for (auto* mesh : mesh_list_)
    delete mesh, mesh = nullptr;
  if (bvh_root_)
    delete bvh_root_, bvh_root_ = nullptr;
}

void Object::AddMaterial(const std::string& mtl_name) {
  material_table_.push_back(std::make_pair(mtl_name, mesh_list_.size()));
}

const std::string& Object::GetMaterialNameByMeshID(size_t mesh_id) const {
  const std::string* mtl_name = &material_table_.front().first;
  for (auto it = material_table_.cbegin() + 1; it != material_table_.cend(); ++it) {
    size_t start_id = it->second;
    if (mesh_id < start_id)
      return *mtl_name;
    else
      mtl_name = &it->first;
  }
  return *mtl_name;
}

const Material& Object::GetMaterialByMeshID(size_t mesh_id) const {
  return scene_->materials().find(GetMaterialNameByMeshID(mesh_id))->second;
}

const Mesh& Object::GetMeshByMeshID(size_t mesh_id) const {
  return *mesh_list_[mesh_id];
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
    assert(false && "No corresponding polygon!");
  }
}

void Object::ConstructBVH() {
  /* constructs the root node */
  Vector4f obj_llb = Vector4f::All(std::numeric_limits<float>::max());
  Vector4f obj_ruf = Vector4f::All(std::numeric_limits<float>::min());
  std::vector<BVH*> bvh_leaves;
  for (size_t i = 1; i < mesh_list_.size(); ++i) {
    Vector4f llb = Vector4f::All(std::numeric_limits<float>::max());
    Vector4f ruf = Vector4f::All(std::numeric_limits<float>::min());
    const Mesh* mesh = mesh_list_[i];
    if (mesh->v_num() == 3) {
      const TriMesh* tri = dynamic_cast<const TriMesh*>(mesh);
      for (size_t i = 0; i < mesh->v_num(); ++i) {
        const Vector3f& v = scene_->v()[tri->v_id[i]];
        llb.x() = v.x() < llb.x() ? v.x() : llb.x();
        llb.y() = v.y() < llb.y() ? v.y() : llb.y();
        llb.z() = v.z() < llb.z() ? v.z() : llb.z();
        ruf.x() = v.x() > ruf.x() ? v.x() : ruf.x();
        ruf.y() = v.y() > ruf.y() ? v.y() : ruf.y();
        ruf.z() = v.z() > ruf.z() ? v.z() : ruf.z();
      }
    } else if (mesh->v_num() == 4) {
      const RectMesh* rect = dynamic_cast<const RectMesh*>(mesh);
      for (size_t i = 0; i < mesh->v_num(); ++i) {
        const Vector3f& v = scene_->v()[rect->v_id[i]];
        llb.x() = v.x() < llb.x() ? v.x() : llb.x();
        llb.y() = v.y() < llb.y() ? v.y() : llb.y();
        llb.z() = v.z() < llb.z() ? v.z() : llb.z();
        ruf.x() = v.x() > ruf.x() ? v.x() : ruf.x();
        ruf.y() = v.y() > ruf.y() ? v.y() : ruf.y();
        ruf.z() = v.z() > ruf.z() ? v.z() : ruf.z();
      }
    }
    llb.w() = 1; ruf.w() = 1;
    bvh_leaves.push_back(new BVH(AABB(llb, ruf), i));
    obj_llb.x() = llb.x() < obj_llb.x() ? llb.x() : obj_llb.x();
    obj_llb.y() = llb.y() < obj_llb.y() ? llb.y() : obj_llb.y();
    obj_llb.z() = llb.z() < obj_llb.z() ? llb.z() : obj_llb.z();
    obj_ruf.x() = ruf.x() > obj_ruf.x() ? ruf.x() : obj_ruf.x();
    obj_ruf.y() = ruf.y() > obj_ruf.y() ? ruf.y() : obj_ruf.y();
    obj_ruf.z() = ruf.z() > obj_ruf.z() ? ruf.z() : obj_ruf.z();
  }
  obj_llb.w() = 1; obj_ruf.w() = 1;

  /* constructs the subnodes */
  bvh_root_ = new BVH(AABB(obj_llb, obj_ruf));
  SplitBVHTree(bvh_leaves, &bvh_root_);
  if (bvh_root_->l_child) {
    UpdateAABB(bvh_root_->l_child->aabb, &bvh_root_);
    if (bvh_root_->r_child)
      UpdateAABB(bvh_root_->r_child->aabb, &bvh_root_);
  }
}

void Object::SplitBVHTree(std::vector<BVH*>& bvh_list, BVH** const bvh_parent) {
  if (bvh_list.size() == 1) {
    delete *bvh_parent, *bvh_parent = nullptr;
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
    Vector4f new_llb = aabb_parent.llb(); new_llb.x() = aabb_parent.ct().x();
    Vector4f new_ruf = aabb_parent.ruf(); new_ruf.x() = aabb_parent.ct().x();
    left_bvh->aabb.Reset(aabb_parent.llb(), new_ruf);
    right_bvh->aabb.Reset(new_llb, aabb_parent.ruf());
  } else if (aabb_parent.diag().y() >= aabb_parent.diag().x() &&
             aabb_parent.diag().y() >= aabb_parent.diag().z()) {
    hyper_plane.y() = 1;
    hyper_plane.w() = -aabb_parent.ct().y();
    Vector4f new_llb = aabb_parent.llb(); new_llb.y() = aabb_parent.ct().y();
    Vector4f new_ruf = aabb_parent.ruf(); new_ruf.y() = aabb_parent.ct().y();
    left_bvh->aabb.Reset(aabb_parent.llb(), new_ruf);
    right_bvh->aabb.Reset(new_llb, aabb_parent.ruf());
  } else {
    hyper_plane.z() = 1;
    hyper_plane.w() = -aabb_parent.ct().z();
    Vector4f new_llb = aabb_parent.llb(); new_llb.z() = aabb_parent.ct().z();
    Vector4f new_ruf = aabb_parent.ruf(); new_ruf.z() = aabb_parent.ct().z();
    left_bvh->aabb.Reset(aabb_parent.llb(), new_ruf);
    right_bvh->aabb.Reset(new_llb, aabb_parent.ruf());
  }

  std::vector<BVH*> left_bvh_list;
  std::vector<BVH*> right_bvh_list;
  while (not bvh_list.empty()) {
    BVH* bvh = bvh_list.back();
    bvh_list.pop_back();
    if (bvh->aabb.ct() * hyper_plane < 0) {
      left_bvh_list.push_back(bvh);
    } else if (bvh->aabb.ct() * hyper_plane > 0) {
      right_bvh_list.push_back(bvh);
    } else {
      if (left_bvh_list.size() < right_bvh_list.size())
        left_bvh_list.push_back(bvh);
      else
        right_bvh_list.push_back(bvh);
    }
  }

  if (not left_bvh_list.empty()) {
    SplitBVHTree(left_bvh_list, &left_bvh);
    (*bvh_parent)->l_child = left_bvh;
    if (not right_bvh_list.empty()) {
      SplitBVHTree(right_bvh_list, &right_bvh);
      (*bvh_parent)->r_child = right_bvh;
    } else {
      delete right_bvh, right_bvh = nullptr;
    }
  } else {
    SplitBVHTree(right_bvh_list, &right_bvh);
    (*bvh_parent)->l_child = right_bvh;
    delete left_bvh, left_bvh = nullptr;
  }

  /* updates AABB */
  if ((*bvh_parent)->l_child) {
    UpdateAABB((*bvh_parent)->l_child->aabb, bvh_parent);
    if ((*bvh_parent)->r_child)
      UpdateAABB((*bvh_parent)->r_child->aabb, bvh_parent);
  }
}

void Object::UpdateAABB(const AABB& new_aabb, BVH** const bvh) {
  AABB& old_aabb = (*bvh)->aabb;
  old_aabb.llb().x() = new_aabb.llb().x() < old_aabb.llb().x() ? new_aabb.llb().x() : old_aabb.llb().x();
  old_aabb.llb().y() = new_aabb.llb().y() < old_aabb.llb().y() ? new_aabb.llb().y() : old_aabb.llb().y();
  old_aabb.llb().z() = new_aabb.llb().z() < old_aabb.llb().z() ? new_aabb.llb().z() : old_aabb.llb().z();
  old_aabb.ruf().x() = new_aabb.ruf().x() > old_aabb.ruf().x() ? new_aabb.ruf().x() : old_aabb.ruf().x();
  old_aabb.ruf().y() = new_aabb.ruf().y() > old_aabb.ruf().y() ? new_aabb.ruf().y() : old_aabb.ruf().y();
  old_aabb.ruf().z() = new_aabb.ruf().z() > old_aabb.ruf().z() ? new_aabb.ruf().z() : old_aabb.ruf().z();

  old_aabb.diag() = Vector3f(old_aabb.ruf() - old_aabb.llb());
  old_aabb.ct() = (old_aabb.llb() + old_aabb.ruf()) / 2;
  old_aabb.ct() /= old_aabb.ct().w();
}
