#pragma once

#include <algorithm>
#include <vector>

#include <cheers/utils/im_export.hpp>

namespace mcpt {

class MaskGrids {
public:
  MaskGrids(unsigned int img_w, std::vector<bool>& mask) : m_image_width(img_w), m_mask(mask) {
    ImGui::BeginChild("Grids");
    m_grid_size = ImGui::GetTextLineHeight();
    m_grids_per_row = (ImGui::GetContentRegionAvail().x + ImGui::GetStyle().ItemSpacing.x) /
                      (ImGui::GetTextLineHeight() + ImGui::GetStyle().ItemSpacing.x);
    if (m_grids_per_row == 0)
      m_grids_per_row = 1;
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, m_grid_size / 4.0F);
  }

  ~MaskGrids() noexcept {
    ImGui::PopStyleVar();
    ImGui::EndChild();
  }

  void Grid(size_t index) const {
    GridStyleGuard style(index, m_mask[index]);
    if (index % m_grids_per_row)
      ImGui::SameLine();
    m_mask[index] = m_mask[index] ^ ImGui::Button("##Enabled", ImVec2(m_grid_size, m_grid_size));
    if (ImGui::IsItemHovered() && ImGui::BeginTooltip()) {
      ImGui::Text("(%lu, %lu)", index % m_image_width, index / m_image_width);
      ImGui::EndTooltip();
    }
  }

private:
  struct GridStyleGuard {
    static constexpr ImVec4 COLOR_EN[]{
        {0.1F, 0.6F, 0.1F, 1.0F}, {0.1F, 0.8F, 0.1F, 1.0F}, {0.1F, 1.0F, 0.1F, 1.0F}};
    static constexpr ImVec4 COLOR_DIS[]{
        {0.2F, 0.2F, 0.2F, 1.0F}, {0.4F, 0.4F, 0.4F, 1.0F}, {0.6F, 0.6F, 0.6F, 1.0F}};

    GridStyleGuard(size_t index, bool enabled) {
      ImGui::PushID(index);
      ImGui::PushStyleColor(ImGuiCol_Button, enabled ? COLOR_EN[0] : COLOR_DIS[0]);
      ImGui::PushStyleColor(ImGuiCol_ButtonHovered, enabled ? COLOR_EN[1] : COLOR_DIS[1]);
      ImGui::PushStyleColor(ImGuiCol_ButtonActive, enabled ? COLOR_EN[2] : COLOR_DIS[2]);
    }

    ~GridStyleGuard() noexcept {
      ImGui::PopStyleColor(3);
      ImGui::PopID();
    }
  };

  unsigned int m_image_width;
  std::vector<bool>& m_mask;

  float m_grid_size;
  size_t m_grids_per_row;
};

inline void ImMaskGrids(
    unsigned int img_w, int& min, int& max, const std::vector<int>& len, std::vector<bool>& mask) {
  ImGui::TextUnformatted("Paths");
  ImGui::SameLine();
  if (ImGui::SmallButton("All")) {
    for (size_t i = 0; i < mask.size(); ++i)
      mask[i] = true;
  }
  ImGui::SameLine();
  if (ImGui::SmallButton("Clear")) {
    for (size_t i = 0; i < mask.size(); ++i)
      mask[i] = false;
  }
  ImGui::SameLine();
  if (ImGui::SmallButton("Invert")) {
    for (size_t i = 0; i < mask.size(); ++i)
      mask[i] = mask[i] ^ 1;
  }
  ImGui::Spacing();

  int max_len = *std::max_element(len.cbegin(), len.cend());
  if (ImGui::DragIntRange2("Length Range", &min, &max, 1.0F, 0, max_len, "Min: %d", "Max: %d")) {
    for (size_t i = 0; i < len.size(); ++i)
      mask[i] = (len[i] >= min && len[i] <= max);
  }
  ImGui::Spacing();

  MaskGrids grids(img_w, mask);
  for (size_t i = 0; i < mask.size(); ++i)
    grids.Grid(i);
}

}  // namespace mcpt
