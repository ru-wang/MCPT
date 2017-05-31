#include "obj_parser.h"

#include "eigen.h"
#include "geometry.h"
#include "object.h"
#include "scene.h"
#include "utils.h"

#include <cassert>
#include <fstream>
#include <sstream>
#include <streambuf>
#include <string>
#include <tuple>

#ifdef VERBOSE
#include <iostream>
#endif

using namespace std;

void OBJParser::LoadOBJ(const string& obj_filename, Scene* scene) {
  assert(scene && "Should create a scene first!");
  ifstream ifs(obj_filename);

  int smooth = 0;
  Object* current_obj = nullptr;
  while (ifs && !ifs.eof()) {
    stringstream ss = Utils::SafelyGetLine(ifs);
    string specifier;
    ss >> specifier;

    if (specifier == "mtllib") {         /* specifies the material library */
      string dir = Utils::RemoveSuffix(Utils::RemoveSuffix(obj_filename), '/');
      string mtl_filename;
      ss >> mtl_filename;
      mtl_filename = dir + "/" + mtl_filename;
#ifdef VERBOSE
      std::cout << "| MTL file: " << mtl_filename << "\n";
#endif
      LoadMTL(mtl_filename, scene);
    } else if (specifier == "g") {       /* specifies the next group name */
      string grp_name;
      ss >> grp_name;
      if (grp_name != "default") {
        auto entry = scene->objects().find(grp_name);
        if (entry == scene->objects().end())
          entry = get<0>(scene->objects().emplace(grp_name, scene));
        current_obj = &entry->second;
        current_obj->set_smooth(smooth);
      } else {
        current_obj = nullptr;
      }
    }

    else if (specifier == "v") {         /* adds a new vertex to the default group */
      assert(current_obj == nullptr && "Wrong .obj file format!");
      scene->v().push_back(Vector3f());
      Vector3f& v = scene->v().back();
      ss >> v[0] >> v[1] >> v[2];
    } else if (specifier == "vt") {      /* adds a new vertex texture to the default group */
      assert(current_obj == nullptr && "Wrong .obj file format!");
      scene->vt().push_back(Vector2f());
      Vector2f& vt = scene->vt().back();
      ss >> vt[0] >> vt[1];
    } else if (specifier == "vn") {      /* adds a new vertex normal to the default group */
      assert(current_obj == nullptr && "Wrong .obj file format!");
      scene->vn().push_back(Vector3f());
      Vector3f& vn = scene->vn().back();
      ss >> vn[0] >> vn[1] >> vn[2];
    }

    else if (specifier == "s") {         /* specifies the smooth term for the next object */
      string s;
      ss >> s;
      if (s == "off")
        smooth = 0;
      else
        smooth = stoi(s);
    } else if (specifier == "usemtl") {  /* specifies the material used for the successive meshes */
      assert(current_obj && "Wrong .obj file format!");
      string mtl_name;
      ss >> mtl_name;
      current_obj->AddMaterial(mtl_name);
    } else if (specifier == "f") {       /* adds a new polygon for the current object */
      assert(current_obj && "Wrong .obj file format!");

      int id;
      vector<int> ids;
      streambuf* sb = ss.rdbuf();
      while (true) {
        char ch = sb->sgetc();
        if (ch >= '0' && ch <= '9')
          ss >> id, ids.push_back(id);
        else if (ch != EOF)
          sb->sbumpc();
        else
          break;
      }

      if (ids.size() / 3 == 3) {         /* it is a triangle */
        TriMesh* triangle = new TriMesh;
        for (size_t i = 0; i < 3; ++i) {
          triangle->v_id[i] = ids[i * 3 + 0];
          triangle->vt_id[i] = ids[i * 3 + 1];
          triangle->vn_id[i] = ids[i * 3 + 2];
        }
        const Vector3f& v1 = scene->v()[triangle->v_id[0]];
        const Vector3f& v2 = scene->v()[triangle->v_id[1]];
        const Vector3f& v3 = scene->v()[triangle->v_id[2]];
        triangle->f = Polygon(v1, v2, v3);
        for (size_t i = 0; i < 3; ++i)
          triangle->n += scene->vn()[triangle->vn_id[i]];
        triangle->n.NormalizeInPlace();
        current_obj->mesh_list().push_back(triangle);
      } else if (ids.size() / 3 == 4) {  /* it is a rectangle */
        RectMesh* rectangle = new RectMesh;
        for (size_t i = 0; i < 4; ++i) {
          rectangle->v_id[i] = ids[i * 3 + 0];
          rectangle->vt_id[i] = ids[i * 3 + 1];
          rectangle->vn_id[i] = ids[i * 3 + 2];
        }
        const Vector3f& v1 = scene->v()[rectangle->v_id[0]];
        const Vector3f& v2 = scene->v()[rectangle->v_id[1]];
        const Vector3f& v3 = scene->v()[rectangle->v_id[2]];
        const Vector3f& v4 = scene->v()[rectangle->v_id[3]];
        rectangle->f = Polygon(v1, v2, v3, v4);
        for (size_t i = 0; i < 4; ++i)
          rectangle->n += scene->vn()[rectangle->vn_id[i]];
        rectangle->n.NormalizeInPlace();
        current_obj->mesh_list().push_back(rectangle);
      }
    }
  }

  ifs.close();
}

void OBJParser::LoadMTL(const string& mtl_filename, Scene* scene) {
  assert(scene && "Should create a scene first!");
  ifstream ifs(mtl_filename);

  Material* current_mtl = nullptr;
  while (ifs && !ifs.eof()) {
    stringstream ss = Utils::SafelyGetLine(ifs);
    string specifier;
    ss >> specifier;

    if (specifier == "newmtl") {        /* adds new material */
      string mtl_name;
      ss >> mtl_name;
      current_mtl = &scene->materials()[mtl_name];
    } else if (specifier == "illum") {  /* specifies the illumination type */
      assert(current_mtl && "Wrong .mtl file format!");
      ss >> current_mtl->illum;
    } else if (specifier == "Kd") {   /* specifies the diffuse reflectivity */
      assert(current_mtl && "Wrong .mtl file format!");
      Vector3f& Kd = current_mtl->Kd;
      ss >> Kd[0] >> Kd[1] >> Kd[2];
    } else if (specifier == "Ka") {   /* specifies the ambient reflectivity */
      assert(current_mtl && "Wrong .mtl file format!");
      Vector3f& Ka = current_mtl->Ka;
      ss >> Ka[0] >> Ka[1] >> Ka[2];
    } else if (specifier == "Ks") {   /* specifies the ambient reflectivity */
      assert(current_mtl && "Wrong .mtl file format!");
      Vector3f& Ks = current_mtl->Ks;
      ss >> Ks[0] >> Ks[1] >> Ks[2];
    } else if (specifier == "Ns") {   /* specifies the specular exponent */
      assert(current_mtl && "Wrong .mtl file format!");
      ss >> current_mtl->Ns;
    } else if (specifier == "Tr") {   /* specifies the transparency value */
      assert(current_mtl && "Wrong .mtl file format!");
      ss >> current_mtl->Tr;
    } else if (specifier == "Ni") {   /* specifies the optical density */
      assert(current_mtl && "Wrong .mtl file format!");
      ss >> current_mtl->Ni;
    }
  }

  ifs.close();
}
