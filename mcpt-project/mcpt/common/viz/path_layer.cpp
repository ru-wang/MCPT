#include "mcpt/common/viz/path_layer.hpp"

#include <cheers/resource/shader_absolute_path.hpp>
#include <cheers/utils/gl_object.hpp>
#include <cheers/utils/im_export.hpp>

#include "mcpt/common/assert.hpp"

namespace mcpt {

void PathLayer::OnCreateRenderer() {
  cheers::ShaderObject vshader(cheers::model_vshader_absolute_path);
  cheers::ShaderObject fshader(cheers::model_fshader_absolute_path);

  add_render_program();
  auto& program = render_program(0);

  program.Attach(vshader, fshader);
  program.Link();
  program.Detach(vshader, fshader);

  program.GenVao(2);
}

void PathLayer::OnDestroyRenderer() {
  clear_render_program();
}

void PathLayer::OnUpdateImFrame() {
  if (ImGui::Begin("Path Properties")) {
    ImGui::Checkbox("Draw Normal", &m_render_data.draw_normal);
    ImGui::DragFloat("Line Width", &m_render_data.line_width, 0.5f, 0.5f, 5.0f, "%.1f");
    ImGui::ColorEdit4("Path", m_render_data.path_color.data());
    ImGui::BeginDisabled(!m_render_data.draw_normal);
    ImGui::ColorEdit4("Normal", m_render_data.normal_color.data());
    ImGui::EndDisabled();
  }
  ImGui::End();
}

void PathLayer::OnUpdateRenderData() {
  std::unique_lock try_lock(render_data_mutex(), std::try_to_lock);
  if (!try_lock.owns_lock())
    return;
  if (!m_render_data.dirty)
    return;

  auto& program = render_program(0);
  auto& normal_vao = program.Vao(0);
  auto& path_vao = program.Vao(1);

  normal_vao.Bind();
  {
    size_t new_vbo_size = m_render_data.normal_data.size();
    size_t old_vbo_size = normal_vao.VboSize();
    DASSERT(new_vbo_size >= old_vbo_size, "wrong normal data VBO size");

    normal_vao.GenVbo(new_vbo_size - old_vbo_size);
    for (size_t i = old_vbo_size; i < new_vbo_size; ++i) {
      normal_vao.Vbo(i).Bind();
      GLsizeiptr data_size = sizeof(float) * m_render_data.normal_data.at(i).size();
      const void* data_ptr = m_render_data.normal_data.at(i).data();
      glBufferData(GL_ARRAY_BUFFER, data_size, data_ptr, GL_STATIC_DRAW);
    }
  }

  path_vao.Bind();
  {
    size_t new_vbo_size = m_render_data.path_data.size();
    size_t old_vbo_size = path_vao.VboSize();
    DASSERT(new_vbo_size >= old_vbo_size, "wrong path data VBO size");

    path_vao.GenVbo(new_vbo_size - old_vbo_size);
    for (size_t i = old_vbo_size; i < new_vbo_size; ++i) {
      path_vao.Vbo(i).Bind();
      GLsizeiptr data_size = sizeof(float) * m_render_data.path_data.at(i).size();
      const void* data_ptr = m_render_data.path_data.at(i).data();
      glBufferData(GL_ARRAY_BUFFER, data_size, data_ptr, GL_STATIC_DRAW);
    }
  }

  m_render_data.dirty = false;
}

void PathLayer::OnRenderLayer(const float* matrix_vp) {
  auto& program = render_program(0);
  program.Use();

  bool depth_test_enabled = glIsEnabled(GL_DEPTH_TEST);
  glDisable(GL_DEPTH_TEST);

  glLineWidth(m_render_data.line_width);
  glUniformMatrix4fv(program.Uniform("mvp"), 1, GL_FALSE, matrix_vp);

  for (size_t i = 0; m_render_data.draw_normal && i < program.Vao(0).VboSize(); ++i) {
    program.Vao(0).Bind();
    program.Vao(0).Vbo(i).Bind();
    program.EnableAttrib("pos");

    GLint64 num_buffer_bytes;
    glGetBufferParameteri64v(GL_ARRAY_BUFFER, GL_BUFFER_SIZE, &num_buffer_bytes);

    glVertexAttribPointer(program.Attrib("pos"), 3, GL_FLOAT, GL_FALSE, 0, 0);
    glUniform4fv(program.Uniform("clr"), 1, m_render_data.normal_color.data());
    glDrawArrays(GL_LINES, 0, num_buffer_bytes / sizeof(float) / 3);
  }

  for (size_t i = 0; i < program.Vao(1).VboSize(); ++i) {
    program.Vao(1).Bind();
    program.Vao(1).Vbo(i).Bind();
    program.EnableAttrib("pos");

    GLint64 num_buffer_bytes;
    glGetBufferParameteri64v(GL_ARRAY_BUFFER, GL_BUFFER_SIZE, &num_buffer_bytes);

    glVertexAttribPointer(program.Attrib("pos"), 3, GL_FLOAT, GL_FALSE, 0, 0);
    glUniform4fv(program.Uniform("clr"), 1, m_render_data.path_color.data());
    glDrawArrays(GL_LINES, 0, num_buffer_bytes / sizeof(float) / 3);
  }

  if (depth_test_enabled)
    glEnable(GL_DEPTH_TEST);
}

}  // namespace mcpt
