#ifndef MCPT_SCENE_H_
#define MCPT_SCENE_H_

#include "brdf.h"
#include "eigen.h"
#include "object.h"

#include <string>
#include <unordered_map>
#include <vector>

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

#endif  /* MCPT_SCENE_H_ */
