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
  struct Blueprint {
    std::string urdfFilepath;
    Magnum::Range3D heatVolume;
    Magnum::Vector3 openSensorPos;
    Magnum::Vector3 openSensorDir;
    float roomTemp = 75.f;
    float targetTemp = 350.f;
    float closedTempPerSec = 10.f;
    float openTempPerSec = -1;
  };

  OvenEntity(esp::sim::Simulator* sim,
             const OvenEntity::Blueprint& bp,
             const Magnum::Vector3& translation,
             const Magnum::Quaternion& rotation);

  ~OvenEntity();

  bool isInsideCookVolume(const Magnum::Vector3& pos);

  void update(float dt);

  void debugRender(esp::gfx::Debug3DText& debug3dText,
                   esp::gfx::DebugRender& debugRender);

  float getTemperature() { return temp_; }

 private:
  Blueprint bp_;
  int objId_ = -1;
  esp::sim::Simulator* sim_ = nullptr;
  float temp_ = 0.f;
  bool isClosed_ = true;
};

}  // namespace scripted
}  // namespace esp

#endif  // ESP_SCRIPTED_OVENENTITY_H_
