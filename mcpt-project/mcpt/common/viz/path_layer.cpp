#include "mcpt/common/viz/path_layer.hpp"

#include <cheers/resource/shader_absolute_path.hpp>
#include <cheers/utils/gl_object.hpp>
#include <cheers/utils/im_export.hpp>

#include "mcpt/common/viz/mask_grids.hpp"

namespace mcpt {

namespace {

void UpdateBuffer(cheers::VertexAttribObject& vao, const std::vector<std::vector<float>>& data) {
  vao.Bind();

  size_t new_vbo_size = data.size();
  size_t old_vbo_size = vao.VboSize();
  vao.GenVbo(new_vbo_size - old_vbo_size);

  for (size_t i = old_vbo_size; i < new_vbo_size; ++i) {
    vao.Vbo(i).Bind();
    GLsizeiptr data_size = sizeof(float) * data[i].size();
    const void* data_ptr = data[i].data();
    glBufferData(GL_ARRAY_BUFFER, data_size, data_ptr, GL_STATIC_DRAW);
  }
}

void DrawBuffer(cheers::ProgramWrapper& program,
                cheers::VertexAttribObject& vao,
                const std::array<float, 4>& color,
                const std::vector<bool>& mask) {
  for (size_t i = 0; i < vao.VboSize(); ++i) {
    if (!mask[i])
      continue;

    vao.Bind();
    vao.Vbo(i).Bind();
    program.EnableAttrib("pos");

    GLint64 num_buffer_bytes;
    glGetBufferParameteri64v(GL_ARRAY_BUFFER, GL_BUFFER_SIZE, &num_buffer_bytes);
    glUniform4fv(program.Uniform("clr"), 1, color.data());

    glVertexAttribPointer(program.Attrib("pos"), 3, GL_FLOAT, GL_FALSE, 0, 0);
    glDrawArrays(GL_LINES, 0, num_buffer_bytes / sizeof(float) / 3);

    GLsizei stride = sizeof(float) * 6;
    const void* offset_ptr = reinterpret_cast<void*>(sizeof(float) * 3);
    glVertexAttribPointer(program.Attrib("pos"), 3, GL_FLOAT, GL_FALSE, stride, offset_ptr);
    glDrawArrays(GL_POINTS, 0, num_buffer_bytes / sizeof(float) / 3 / 2);
  }
}

}  // namespace

void PathLayer::OnCreateRenderer() {
  cheers::ShaderObject vshader(cheers::model_vshader_absolute_path);
  cheers::ShaderObject fshader(cheers::model_fshader_absolute_path);

  add_render_program();
  auto& program = render_program(0);

  program.Attach(vshader, fshader);
  program.Link();
  program.Detach(vshader, fshader);

  program.GenVao(3);
}

void PathLayer::OnDestroyRenderer() {
  clear_render_program();
}

void PathLayer::OnUpdateImFrame() {
  ImGui::Begin("Path Properties");

  ImGui::Checkbox("Fluoroscopy", &m_render_data.fluoroscopy);
  ImGui::DragFloat("Line Width", &m_render_data.line_width, 0.5f, 0.5f, 5.0f, "%.1f");

  ImGui::Checkbox("##Normal", &m_render_data.draw_normal);
  ImGui::BeginDisabled(!m_render_data.draw_normal);
  ImGui::SameLine();
  ImGui::ColorEdit4("Normal", m_render_data.normal_color.data());
  ImGui::EndDisabled();

  ImGui::Checkbox("##Reverse Path", &m_render_data.draw_rpath);
  ImGui::BeginDisabled(!m_render_data.draw_rpath);
  ImGui::SameLine();
  ImGui::ColorEdit4("Reverse Path", m_render_data.rpath_color.data());
  ImGui::EndDisabled();

  ImGui::Checkbox("##Light Path", &m_render_data.draw_lpath);
  ImGui::BeginDisabled(!m_render_data.draw_lpath);
  ImGui::SameLine();
  ImGui::ColorEdit4("Light Path", m_render_data.lpath_color.data());
  ImGui::EndDisabled();

  ImGui::Spacing();
  ImGui::Separator();
  ImGui::Spacing();

  ImMaskGrids(m_image_width,
              m_render_data.filter_range_min,
              m_render_data.filter_range_max,
              m_render_data.length_data,
              m_render_data.mask_data);

  ImGui::End();
}

void PathLayer::OnUpdateRenderData() {
  std::unique_lock try_lock(render_data_mutex(), std::try_to_lock);
  if (!try_lock.owns_lock())
    return;
  if (!m_render_data.dirty)
    return;
  auto& program = render_program(0);
  UpdateBuffer(program.Vao(0), m_render_data.normal_data);
  UpdateBuffer(program.Vao(1), m_render_data.rpath_data);
  UpdateBuffer(program.Vao(2), m_render_data.lpath_data);
  m_render_data.dirty = false;
}

void PathLayer::OnRenderLayer(const float* matrix_vp) {
  auto& program = render_program(0);
  program.Use();

  bool depth_test_enabled = glIsEnabled(GL_DEPTH_TEST);
  if (m_render_data.fluoroscopy)
    glDisable(GL_DEPTH_TEST);

  glPointSize(m_render_data.line_width * 2.0F);
  glLineWidth(m_render_data.line_width);
  glUniformMatrix4fv(program.Uniform("mvp"), 1, GL_FALSE, matrix_vp);
  if (m_render_data.draw_normal)
    DrawBuffer(program, program.Vao(0), m_render_data.normal_color, m_render_data.mask_data);
  if (m_render_data.draw_rpath)
    DrawBuffer(program, program.Vao(1), m_render_data.rpath_color, m_render_data.mask_data);
  if (m_render_data.draw_lpath)
    DrawBuffer(program, program.Vao(2), m_render_data.lpath_color, m_render_data.mask_data);

  if (m_render_data.fluoroscopy && depth_test_enabled)
    glEnable(GL_DEPTH_TEST);
}

}  // namespace mcpt
