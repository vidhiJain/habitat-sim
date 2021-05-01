// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Debug3DText.h"

#include <Magnum/ImGuiIntegration/Integration.h>
#include <Magnum/Math/Matrix4.h>

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {
namespace gfx {

void Debug3DText::addText(std::string&& text,
                          const Magnum::Vector3& pos,
                          const Magnum::Color4& color) {
  records_.push_back(TextRecord{text, pos, color});
}

// Call this alongside other calls to ImGui::Begin/End, between
// imgui_.newFrame() and imgui_.drawFrame()
void Debug3DText::flushToImGui(const Magnum::Matrix4& camProj,
                               const Magnum::Vector2i& displaySize) {
  auto windowSize = ImGui::GetWindowSize();

  static ImVec2 padding{2, 0};
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, padding);
  static float rounding = 4.f;
  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, rounding);
  static float borderSize = 0.f;
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, borderSize);
  static ImVec2 minSize{1, 1};
  ImGui::PushStyleVar(ImGuiStyleVar_WindowMinSize, minSize);

  for (int i = 0; i < records_.size(); i++) {
    const auto& record = records_[i];

    Mn::Vector4 posProjHomog = camProj * Mn::Vector4(record.pos, 1.f);
    constexpr float eps = 0.001;
    if (posProjHomog.w() < eps) {
      continue;
    }
    Mn::Vector3 posProj = posProjHomog.xyz() / posProjHomog.w();
    // perf todo: bake this last transformation into our Matrix4
    // (camProjViewport)
    Mn::Vector2 posScreen =
        (posProj.xy() * Mn::Vector2(0.5, -0.5) + Mn::Vector2(0.5, 0.5)) *
        Mn::Vector2(displaySize);

    // Nonlinear distance-based scaling. We want far text to be slightly smaller
    // than near text.
    static float minFontScale = 1.0;
    static float fontRange = 1.0;
    static float distanceFactor =
        0.02;  // make larger to make text shrinking more aggressive
    const float fontScale =
        Mn::Math::clamp(float(1.0 - posProj.z()) * (fontRange / distanceFactor),
                        0.f, fontRange) +
        minFontScale;
    CORRADE_INTERNAL_ASSERT(fontScale >= minFontScale &&
                            fontScale <= minFontScale + fontRange);

    static Mn::Vector2 screenPosOffset = Mn::Vector2(-7, -15);
    posScreen += screenPosOffset;

    ImGui::SetNextWindowPos(ImVec2(posScreen.x(), posScreen.y()));
    // perf todo: avoid constructing so many std::strings
    static float bgAlpha = 0.5;
    ImGui::SetNextWindowBgAlpha(bgAlpha);
    const auto windowName = "Debug3DText" + std::to_string(i);
    ImGui::Begin(
        windowName.c_str(), NULL,
        //  ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground |
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::SetWindowFontScale(fontScale);
    ImGui::PushStyleColor(ImGuiCol_Text,
                          ImVec4(record.color.r(), record.color.g(),
                                 record.color.b(), record.color.a()));
    ImGui::TextUnformatted(record.text.c_str());
    ImGui::PopStyleColor();
    ImGui::End();
  }

  ImGui::PopStyleVar();
  ImGui::PopStyleVar();
  ImGui::PopStyleVar();
  ImGui::PopStyleVar();

  records_.clear();
}

}  // namespace gfx
}  // namespace esp
