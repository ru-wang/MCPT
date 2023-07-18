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
};

struct MeshIndexGroup {
  std::string material;
  std::vector<MeshIndex> mesh_index;
};

struct Mesh {
  std::string material;
  ConvexPolygon<float> polygon;
  Polygon2D<float> text_coords;
  Eigen::Vector3f normal;
};

inline bool operator==(const MeshIndex& lhs, const MeshIndex& rhs) {
  return lhs.vindex == rhs.vindex && lhs.tindex == rhs.tindex && lhs.nindex == rhs.nindex;
}

inline bool operator==(const MeshIndexGroup& lhs, const MeshIndexGroup& rhs) {
  return lhs.material == rhs.material && lhs.mesh_index == rhs.mesh_index;
}

}  // namespace mcpt
