#pragma once

#include <string>
#include <vector>

#include <Eigen/Eigen>

#include "mcpt/common/geometry/types.hpp"

namespace mcpt {

struct MeshIndex {
  // index for one mesh should have same length
  // index ranges start from zero which is different with the `.obj' file
  std::vector<size_t> vindex;
  std::vector<size_t> tindex;
  std::vector<size_t> nindex;

  friend bool operator==(const MeshIndex& lhs, const MeshIndex& rhs) {
    return lhs.vindex == rhs.vindex && lhs.tindex == rhs.tindex && lhs.nindex == rhs.nindex;
  }
};

struct MeshIndexGroup {
  int smooth = 0;
  std::string material;
  std::vector<MeshIndex> mesh_index;

  friend bool operator==(const MeshIndexGroup& lhs, const MeshIndexGroup& rhs) {
    return lhs.smooth     == rhs.smooth &&
           lhs.material   == rhs.material &&
           lhs.mesh_index == rhs.mesh_index;
  }
};

struct Mesh {
  std::string material;
  ConvexPolygon<float> polygon;
  Eigen::Vector3f normal;
};

}  // namespace mcpt
