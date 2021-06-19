// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_ARRANGERECORDER_ARRANGER_H_
#define ESP_ARRANGERECORDER_ARRANGER_H_

#include "esp/gfx/DebugRender.h"
#include "esp/gfx/RenderCamera.h"
#include "esp/sim/Simulator.h"

namespace esp {
namespace arrange_recorder {

class Arranger {
 public:
  Arranger(esp::sim::Simulator* simulator,
           esp::gfx::RenderCamera* renderCamera,
           esp::gfx::DebugRender* debugRender);

  void setCursor(const Magnum::Vector2i& cursor) { cursor_ = cursor; }

  void update(float dt, bool isPrimaryButton, bool isSecondaryButton);

 private:
  struct LinkAnimation {
    int artObjId = -1;
    int linkId = -1;
    int jointPosOffset = -1;
    float startPos = -1.f;
    float endPos = -1.f;
    float animTimer = 0.f;
    float animDuration = -1.f;
  };

  int getNumRotationIndices();

  Mn::Quaternion getRotationByIndex(int index);

  esp::sim::Simulator* simulator_ = nullptr;
  esp::gfx::RenderCamera* renderCamera_ = nullptr;
  esp::gfx::DebugRender* debugRender_ = nullptr;
  Magnum::Vector2i cursor_;
  std::vector<int> existingObjectIds_;
  int heldObjId_ = -1;
  int recentHeldObjRotIndex_ = 0;
  Corrade::Containers::Optional<LinkAnimation> linkAnimOpt_;
};

}  // namespace arrange_recorder
}  // namespace esp

#endif
