#include "mcpt/common/viz/object_layer.hpp"

#include <array>
#include <vector>

#include <GL/glew.h>

#include <cheers/resource/shader_absolute_path.hpp>

namespace mcpt {

namespace {

constexpr std::array CAMERA_SHAPE = {
     0.0F,  0.0F, 0.0F,  // center
    +1.0F, +0.5F, 2.0F,  // right-bottom
    +1.0F, -0.5F, 2.0F,  // right-top
    -1.0F, -0.5F, 2.0F,  // left-top
    -1.0F, +0.5F, 2.0F,  // left-bottom
    +1.0F, +0.5F, 2.0F,  // right-bottom
};

constexpr std::array CAMERA_UP_SHAPE = {
     0.0F, -1.1F, 2.0F,  // center-top
    +1.0F, -0.6F, 2.0F,  // right-bottom
    -1.0F, -0.6F, 2.0F,  // left-bottom
};

constexpr std::array FACET_COLOR{0.22F, 0.22F, 0.22F, 0.78F};
constexpr std::array WIRE_COLOR{0.1F, 0.1F, 0.1F, 1.0F};
constexpr std::array CAMERA_COLOR{0.1F, 0.1F, 0.1F, 1.0F};
constexpr std::array CAMERA_UP_COLOR{0.9F, 0.1F, 0.1F, 1.0F};

}  // namespace

void ObjectLayer::OnCreateRenderer() {
  cheers::ShaderObject vshader(cheers::model_vshader_absolute_path);
  cheers::ShaderObject fshader(cheers::model_fshader_absolute_path);

  add_render_program();
  auto& program = render_program(0);

  program.Attach(vshader, fshader);
  program.Link();
  program.Detach(vshader, fshader);

  program.GenVao(1);
  program.Vao(0).GenVbo(1);
  program.Vao(0).GenEbo(1);

  // for camera shapes
  program.Vao(0).GenVbo(2);
  program.Vao(0).Bind();

  program.Vao(0).Vbo(1).Bind();
  {
    GLsizeiptr data_size = sizeof(float) * CAMERA_SHAPE.size();
    const void* data_ptr = CAMERA_SHAPE.data();
    glBufferData(GL_ARRAY_BUFFER, data_size, data_ptr, GL_STATIC_DRAW);
  }

  program.Vao(0).Vbo(2).Bind();
  {
    GLsizeiptr data_size = sizeof(float) * CAMERA_UP_SHAPE.size();
    const void* data_ptr = CAMERA_UP_SHAPE.data();
    glBufferData(GL_ARRAY_BUFFER, data_size, data_ptr, GL_STATIC_DRAW);
  }
}

void ObjectLayer::OnDestroyRenderer() {
  clear_render_program();
}

void ObjectLayer::OnUpdateRenderData() {
  std::unique_lock try_lock(render_data_mutex(), std::try_to_lock);
  if (!try_lock.owns_lock())
    return;
  if (!m_render_data.dirty)
    return;

  auto& program = render_program(0);
  auto& vao = program.Vao(0);

  vao.Bind();
  vao.Vbo(0).Bind();
  {
    std::vector<float> vertex_data;
    for (const auto& vertex : m_render_data.object.vertices())
      vertex_data.insert(vertex_data.cend(), vertex.begin(), vertex.end());

    GLsizeiptr data_size = sizeof(float) * vertex_data.size();
    const void* data_ptr = vertex_data.data();
    glBufferData(GL_ARRAY_BUFFER, data_size, data_ptr, GL_STATIC_DRAW);
  }

  vao.Ebo(0).Bind();
  {
    std::vector<unsigned short> index_data;
    for (const auto& mesh_group : m_render_data.object.mesh_groups()) {
      for (const auto& mesh_index : mesh_group.mesh_index) {
        // convert to triangle fans
        for (size_t i = 1, j = 2; j < mesh_index.vindex.size(); ++i, ++j) {
          index_data.push_back(mesh_index.vindex.at(0));
          index_data.push_back(mesh_index.vindex.at(i));
          index_data.push_back(mesh_index.vindex.at(j));
        }
      }
    }

    GLsizeiptr data_size = sizeof(unsigned short) * index_data.size();
    const void* data_ptr = index_data.data();
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, data_size, data_ptr, GL_STATIC_DRAW);

    m_render_data.num_indices = index_data.size();
  }

  m_render_data.dirty = false;
}

void ObjectLayer::OnRenderLayer(const float* matrix_vp) {
  auto& program = render_program(0);

  program.Use();
  program.Vao(0).Bind();
  program.EnableAttrib("pos");

  glLineWidth(2.0F);

  {
    glUniformMatrix4fv(program.Uniform("mvp"), 1, GL_FALSE, matrix_vp);

    program.Vao(0).Vbo(0).Bind();
    program.Vao(0).Ebo(0).Bind();
    glVertexAttribPointer(program.Attrib("pos"), 3, GL_FLOAT, GL_FALSE, 0, 0);

    glUniform4fv(program.Uniform("clr"), 1, FACET_COLOR.data());
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glDrawElements(GL_TRIANGLES, m_render_data.num_indices, GL_UNSIGNED_SHORT, 0);

    glUniform4fv(program.Uniform("clr"), 1, WIRE_COLOR.data());
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glDrawElements(GL_TRIANGLES, m_render_data.num_indices, GL_UNSIGNED_SHORT, 0);
  }

  {
    Eigen::Map<const Eigen::Matrix4f> vp_matrix(matrix_vp);
    Eigen::Matrix4f mvp_matrix = vp_matrix * m_render_data.camera_pose;
    glUniformMatrix4fv(program.Uniform("mvp"), 1, GL_FALSE, mvp_matrix.data());

    {
      program.Vao(0).Vbo(1).Bind();
      glVertexAttribPointer(program.Attrib("pos"), 3, GL_FLOAT, GL_FALSE, 0, 0);

      glUniform4fv(program.Uniform("clr"), 1, CAMERA_COLOR.data());
      glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
      glDrawArrays(GL_TRIANGLE_FAN, 0, CAMERA_SHAPE.size() / 3);
    }

    {
      program.Vao(0).Vbo(2).Bind();
      glVertexAttribPointer(program.Attrib("pos"), 3, GL_FLOAT, GL_FALSE, 0, 0);

      glUniform4fv(program.Uniform("clr"), 1, CAMERA_COLOR.data());
      glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
      glDrawArrays(GL_TRIANGLES, 0, CAMERA_UP_SHAPE.size() / 3);

      glUniform4fv(program.Uniform("clr"), 1, CAMERA_UP_COLOR.data());
      glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
      glDrawArrays(GL_TRIANGLES, 0, CAMERA_UP_SHAPE.size() / 3);
    }
  }
}

}  // namespace mcpt
