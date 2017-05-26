#include "monte_carlo.h"
#include "obj_parser.h"
#include "scene.h"
#include "utils.h"

#include <cassert>
#include <fstream>
#include <queue>
#include <string>

#include <iomanip>
#include <iostream>

using namespace std;

namespace {

void BFS(const Object::BVH* const root) {
  size_t nonleaves = 0;
  size_t leaves = 0;
  size_t invalid = 0;

  queue<const Object::BVH*> queue;
  queue.push(root);
  while (not queue.empty()) {
    const Object::BVH* current_bvh = queue.front();
    queue.pop();

    if (current_bvh->nonleaf()) {
      queue.push(current_bvh->l_child);
      assert(current_bvh->aabb.diag().L2() >= queue.back()->aabb.diag().L2());
      assert(current_bvh->aabb.Envelop(queue.back()->aabb.llb()));
      assert(current_bvh->aabb.Envelop(queue.back()->aabb.ruf()));
      if (current_bvh->r_child) {
        queue.push(current_bvh->r_child);
        assert(current_bvh->aabb.diag().L2() >= queue.back()->aabb.diag().L2());
        assert(current_bvh->aabb.Envelop(queue.back()->aabb.llb()));
        assert(current_bvh->aabb.Envelop(queue.back()->aabb.ruf()));
      }
    }

    if (current_bvh->leaf())
      ++leaves;
    else if (current_bvh->nonleaf())
      ++nonleaves;
    else
      ++invalid;
  }
  std::cerr << "| Nonleaf nodes: " << setw(4) << nonleaves << "\n"
            << "| Leaf nodes:    " << setw(4) << leaves << "\n"
            << "| Invalid nodes: " << setw(4) << invalid << "\n";
}

const int w = 40, h = 30;
const Matrix4f MVP_inv = Matrix4f::Identity();
const int k_max = 10, n_max = 100;
const int rays_per_pixel = 1;

}

int main(int argc, char* argv[]) {
  assert(argc > 1);

  std::cerr << "\n"
            << "+++++++++++++++++++++ Reports +++++++++++++++++++++\n";

  string obj_name = argv[1];
  string out_name = Utils::RemoveSuffix(obj_name) + ".ppm";

  std::cerr << "| OBJ file: " << obj_name << "\n";

  Scene scene;
  OBJParser::LoadOBJ(obj_name, &scene);

  std::cerr << "| Out file: " << out_name << "\n"
            << "+                    _________                    |\n"
            << "+                        -                        |\n"
            << "| Object:   " << setw(8) << scene.objects().size() << "\n"
            << "| Material: " << setw(8) << scene.materials().size() << "\n"
            << "| Vertex:   " << setw(8) << scene.v().size() << "\n"
            << "| Texture:  " << setw(8) << scene.vt().size() << "\n"
            << "| Normal:   " << setw(8) << scene.vn().size() << "\n"
            << "+                    _________                    |\n"
            << "+                        -                        |\n";

  scene.Initialize();

  for (const auto& entry : scene.objects()) {
    std::cerr << "+                    _________                    |\n"
              << "+                        -                        |\n"
              << "| Object name:   [" << entry.first << "]\n";
    BFS(entry.second.bvh_root());
  }

  std::cerr << "+                        -                        |\n"
            << "+                    -* End *-                    +\n\n";

  return 0;
}
