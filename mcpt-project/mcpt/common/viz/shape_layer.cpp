#include "mcpt/common/viz/shape_layer.hpp"

#include <cheers/resource/shader_absolute_path.hpp>
#include <cheers/utils/gl_object.hpp>
#include <cheers/utils/im_export.hpp>

#include "mcpt/common/assert.hpp"

namespace mcpt {

void ShapeLayer::OnCreateRenderer() {
  cheers::ShaderObject vshader(cheers::model_vshader_absolute_path);
  cheers::ShaderObject fshader(cheers::model_fshader_absolute_path);

  add_render_program();
  auto& program = render_program(0);

  program.Attach(vshader, fshader);
  program.Link();
  program.Detach(vshader, fshader);

  program.GenVao(2);
}

void ShapeLayer::OnDestroyRenderer() {
  clear_render_program();
}

void ShapeLayer::OnUpdateImFrame() {
  if (ImGui::Begin("Shape Properties")) {
    ImGui::DragFloat("Point Size", &m_render_data.point_size, 0.5f, 0.5f, 5.0f, "%.1f");
    ImGui::DragFloat("Line Width", &m_render_data.line_width, 0.5f, 0.5f, 5.0f, "%.1f");

    ImGui::Checkbox("##Point", &m_render_data.draw_point);
    ImGui::BeginDisabled(!m_render_data.draw_point);
    ImGui::SameLine();
    ImGui::ColorEdit4("Point", m_render_data.point_color.data());
    ImGui::EndDisabled();

    ImGui::Checkbox("##Line", &m_render_data.draw_line);
    ImGui::BeginDisabled(!m_render_data.draw_line);
    ImGui::SameLine();
    ImGui::ColorEdit4("Line", m_render_data.line_color.data());
    ImGui::EndDisabled();
  }
  ImGui::End();
}

void ShapeLayer::OnUpdateRenderData() {
  std::unique_lock try_lock(render_data_mutex(), std::try_to_lock);
  if (!try_lock.owns_lock())
    return;
  if (!m_render_data.dirty)
    return;

  auto& program = render_program(0);
  auto& point_vao = program.Vao(0);
  auto& line_vao = program.Vao(1);

  point_vao.Bind();
  {
    size_t new_vbo_size = m_render_data.point_data.size();
    size_t old_vbo_size = point_vao.VboSize();
    DASSERT(new_vbo_size >= old_vbo_size, "wrong point data VBO size");

    point_vao.GenVbo(new_vbo_size - old_vbo_size);
    for (size_t i = old_vbo_size; i < new_vbo_size; ++i) {
      point_vao.Vbo(i).Bind();
      GLsizeiptr data_size = sizeof(float) * m_render_data.point_data.at(i).size();
      const void* data_ptr = m_render_data.point_data.at(i).data();
      glBufferData(GL_ARRAY_BUFFER, data_size, data_ptr, GL_STATIC_DRAW);
    }
  }

  line_vao.Bind();
  {
    size_t new_vbo_size = m_render_data.line_data.size();
    size_t old_vbo_size = line_vao.VboSize();
    DASSERT(new_vbo_size >= old_vbo_size, "wrong path data VBO size");

    line_vao.GenVbo(new_vbo_size - old_vbo_size);
    for (size_t i = old_vbo_size; i < new_vbo_size; ++i) {
      line_vao.Vbo(i).Bind();
      GLsizeiptr data_size = sizeof(float) * m_render_data.line_data.at(i).size();
      const void* data_ptr = m_render_data.line_data.at(i).data();
      glBufferData(GL_ARRAY_BUFFER, data_size, data_ptr, GL_STATIC_DRAW);
    }
  }

  m_render_data.dirty = false;
}

void ShapeLayer::OnRenderLayer(const float* matrix_vp) {
  auto& program = render_program(0);
  program.Use();

  glPointSize(m_render_data.point_size);
  glLineWidth(m_render_data.line_width);
  glUniformMatrix4fv(program.Uniform("mvp"), 1, GL_FALSE, matrix_vp);

  for (size_t i = 0; m_render_data.draw_point && i < program.Vao(0).VboSize(); ++i) {
    program.Vao(0).Bind();
    program.Vao(0).Vbo(i).Bind();
    program.EnableAttrib("pos");

    GLint64 num_buffer_bytes;
    glGetBufferParameteri64v(GL_ARRAY_BUFFER, GL_BUFFER_SIZE, &num_buffer_bytes);
    glVertexAttribPointer(program.Attrib("pos"), 3, GL_FLOAT, GL_FALSE, 0, 0);
    glUniform4fv(program.Uniform("clr"), 1, m_render_data.point_color.data());
    glDrawArrays(GL_POINTS, 0, num_buffer_bytes / sizeof(float) / 3);
  }

  for (size_t i = 0; m_render_data.draw_line && i < program.Vao(1).VboSize(); ++i) {
    program.Vao(1).Bind();
    program.Vao(1).Vbo(i).Bind();
    program.EnableAttrib("pos");

    GLint64 num_buffer_bytes;
    glGetBufferParameteri64v(GL_ARRAY_BUFFER, GL_BUFFER_SIZE, &num_buffer_bytes);
    glVertexAttribPointer(program.Attrib("pos"), 3, GL_FLOAT, GL_FALSE, 0, 0);
    glUniform4fv(program.Uniform("clr"), 1, m_render_data.line_color.data());
    glDrawArrays(GL_POINTS, 0, num_buffer_bytes / sizeof(float) / 3);
    glDrawArrays(GL_LINES, 0, num_buffer_bytes / sizeof(float) / 3);
  }
}

}  // namespace mcpt
