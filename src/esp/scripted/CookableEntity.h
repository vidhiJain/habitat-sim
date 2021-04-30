// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SCRIPTED_COOKIEENTITY_H_
#define ESP_SCRIPTED_COOKIEENTITY_H_

#include <Magnum/Magnum.h>
#include <Magnum/Math/Quaternion.h>
#include <Magnum/Math/Vector3.h>
#include <esp/gfx/Debug3DText.h>
#include <esp/gfx/DebugRender.h>
#include "esp/sim/Simulator.h"

namespace esp {
namespace scripted {

class CookableEntity {
 public:
  CookableEntity(esp::sim::Simulator* sim,
                 const Magnum::Vector3& translation,
                 const Magnum::Quaternion& rotation);

  ~CookableEntity();

  void update(float dt);

  void debugRender(esp::gfx::Debug3DText& debug3dText,
                   esp::gfx::DebugRender& debugRender);

 private:
  int objId_ = -1;
  esp::sim::Simulator* sim_ = nullptr;
  float cookTime_ = 0.f;
};

}  // namespace scripted
}  // namespace esp

#endif
