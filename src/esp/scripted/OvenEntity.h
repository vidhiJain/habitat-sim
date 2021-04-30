// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SCRIPTED_OVENENTITY_H_
#define ESP_SCRIPTED_OVENENTITY_H_

#include <Magnum/Magnum.h>
#include <Magnum/Math/Quaternion.h>
#include <Magnum/Math/Vector3.h>
#include <esp/gfx/Debug3DText.h>
#include <esp/gfx/DebugRender.h>
#include "esp/sim/Simulator.h"

namespace esp {
namespace scripted {

class OvenEntity {
 public:
  OvenEntity(esp::sim::Simulator* sim,
             const Magnum::Vector3& translation,
             const Magnum::Quaternion& rotation);

  ~OvenEntity();

  bool isInsideCookVolume(const Magnum::Vector3& pos);

  void update(float dt);

  void debugRender(esp::gfx::Debug3DText& debug3dText,
                   esp::gfx::DebugRender& debugRender);

  float getTemperature() { return temp_; }

 private:
  int objId_ = -1;
  esp::sim::Simulator* sim_ = nullptr;
  float temp_ = 75.f;
  Magnum::Range3D heatVolume_;
};

}  // namespace scripted
}  // namespace esp

#endif  // ESP_SCRIPTED_OVENENTITY_H_
