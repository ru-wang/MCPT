#ifndef MCPT_OBJ_PARSER_H_
#define MCPT_OBJ_PARSER_H_

#include <string>

class Scene;

class OBJParser {
 public:
  static void LoadOBJ(const std::string& obj_filename, Scene* scene);
  static void LoadMTL(const std::string& mtl_filename, Scene* scene);
};

#endif  /* MCPT_OBJ_PARSER_H_ */
