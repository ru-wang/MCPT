#ifndef MCPT_PATH_TRACER_H_
#define MCPT_PATH_TRACER_H_

#include <istream>
#include <sstream>
#include <string>

class Scene;

class OBJParser {
 public:
  static void LoadOBJ(const std::string& obj_filename, Scene* scene);
  static void LoadMTL(const std::string& mtl_filename, Scene* scene);

  static std::stringstream SafelyGetLine(std::istream& is);
};

#endif  /* MCPT_PATH_TRACER_H_ */
