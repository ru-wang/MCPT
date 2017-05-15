#include "obj_parser.h"
#include "geometry.h"

#include <cassert>
#include <fstream>
#include <istream>
#include <sstream>
#include <streambuf>
#include <string>

using namespace std;

void OBJParser::LoadOBJ(const string& obj_filename, Scene* scene) {
  assert(scene != nullptr);
  ifstream ifs(obj_filename);

  Object* current_obj = nullptr;
  while (ifs) {
    stringstream ss = SafelyGetLine(ifs);
    string specifier;
    ss >> specifier;

    if (specifier == "mtllib") {         /* specifies the material library */
      string mtl_filename;
      ss >> mtl_filename;
      LoadMTL(mtl_filename, scene);
    } else if (specifier == "g") {       /* specifies the next group name */
      string grp_name;
      ss >> grp_name;
      if (grp_name != "default")
        current_obj = &scene->objects()[grp_name];
      else
        current_obj = nullptr;
    }

    else if (specifier == "v") {         /* adds a new vertex to the default group */
      assert(current_obj == nullptr);
      scene->v().push_back(Vector3f());
      Vector3f& v = scene->v().back();
      ss >> v[0] >> v[1] >> v[2];
    } else if (specifier == "vt") {      /* adds a new vertex texture to the default group */
      assert(current_obj == nullptr);
      scene->vt().push_back(Vector2f());
      Vector2f& vt = scene->vt().back();
      ss >> vt[0] >> vt[1];
    } else if (specifier == "vn") {      /* adds a new vertex normal to the default group */
      assert(current_obj == nullptr);
      scene->vn().push_back(Vector3f());
      Vector3f& vn = scene->vn().back();
      ss >> vn[0] >> vn[1] >> vn[2];
    }

    else if (specifier == "s") {         /* specifies the smooth term for the next object */
      assert(current_obj != nullptr);
      string s;
      ss >> s;
      if (s == "off")
        current_obj->set_smooth(0);
      else
        current_obj->set_smooth(stoi(s));
    } else if (specifier == "usemtl") {  /* specifies the material used for the next object */
      assert(current_obj != nullptr);
      string mtl_name;
      ss >> mtl_name;
      current_obj->set_material(mtl_name);
    } else if (specifier == "f") {       /* adds a new facet for the current object */
      assert(current_obj != nullptr);
      vector<int>& v_id = current_obj->v_id();
      vector<int>& vt_id = current_obj->vt_id();
      vector<int>& vn_id = current_obj->vn_id();

      int id;
      vector<int> ids;
      streambuf* sb = ss.rdbuf();
      while (ss) {
        char ch = sb->sgetc();
        if (ch >= '0' && ch <= '9')
          ss >> id, ids.push_back(id);
        else if (ch != EOF)
          sb->sbumpc();
        else
          break;
      }

      for (auto it = ids.cbegin(); it != ids.cend(); it += 3) {
        v_id.push_back(*it);
        vt_id.push_back(*(it + 1));
        vn_id.push_back(*(it + 2));
      }

      if (current_obj->v_stride() == 0)
        current_obj->set_v_stride(v_id.size());
    }
  }

  ifs.close();
}

void OBJParser::LoadMTL(const string& mtl_filename, Scene* scene) {
  assert(scene != nullptr);
  ifstream ifs(mtl_filename);

  Material* current_mtl = nullptr;
  while (ifs) {
    stringstream ss = SafelyGetLine(ifs);
    string specifier;
    ss >> specifier;

    if (specifier == "newmtl") {      /* adds new material */
      string mtl_name;
      ss >> mtl_name;
      current_mtl = &scene->materials()[mtl_name];
    }

    else if (specifier == "illum") {  /* specifies the illumination type */
      assert(current_mtl != nullptr);
      ss >> current_mtl->illum;
    } else if (specifier == "Kd") {   /* specifies the diffuse reflectivity */
      assert(current_mtl != nullptr);
      Vector3f& Kd = current_mtl->Kd;
      ss >> Kd[0] >> Kd[1] >> Kd[2];
    } else if (specifier == "Ka") {   /* specifies the ambient reflectivity */
      assert(current_mtl != nullptr);
      Vector3f& Ka = current_mtl->Ka;
      ss >> Ka[0] >> Ka[1] >> Ka[2];
    } else if (specifier == "Ns") {   /* specifies the specular exponent */
      assert(current_mtl != nullptr);
      ss >> current_mtl->Ns;
    } else if (specifier == "Tr") {   /* specifies the transparency value */
      assert(current_mtl != nullptr);
      ss >> current_mtl->Tr;
    } else if (specifier == "Ni") {   /* specifies the optical density */
      assert(current_mtl != nullptr);
      ss >> current_mtl->Ni;
    }
  }

  ifs.close();
}

stringstream OBJParser::SafelyGetLine(istream& is) {
  stringstream ss;
  streambuf* sb = is.rdbuf();
  while (true) {
    char ch = sb->sbumpc();
    switch (ch) {
      case '\n': return ss;
      case '\r': if (sb->sgetc() == '\n')
                   sb->sbumpc();
                 return ss;
      case EOF:  if (ss.rdbuf()->in_avail() == 0)
                   is.setstate(istream::eofbit);
                 return ss;
      default:   ss << ch;
    }
  }
}
