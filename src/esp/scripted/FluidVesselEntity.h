// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SCRIPTED_FLUIDVESSELENTITY_H_
#define ESP_SCRIPTED_FLUIDVESSELENTITY_H_

#include <Magnum/Magnum.h>
#include <Magnum/Math/Quaternion.h>
#include <Magnum/Math/Vector3.h>
#include <Corrade/Containers/Optional.h>
#include <esp/gfx/Debug3DText.h>
#include <esp/gfx/DebugRender.h>
#include "esp/sim/Simulator.h"

namespace esp {
namespace scripted {

class FluidVesselEntity {
 public:
  struct Blueprint {
    std::string objHandle;
    Magnum::Vector3 spoutPos;
    Magnum::Vector3 spoutDir;
    float spoutRadius;
    float volume; // in liters, or 0 to indicate infinite volume
    std::string initialFluidType; // or empty string
  };

  FluidVesselEntity(esp::sim::Simulator* sim,
             const FluidVesselEntity::Blueprint& bp,
             const Magnum::Vector3& translation,
             const Magnum::Quaternion& rotation);

  ~FluidVesselEntity();

  void update(float dt);

  void debugRender(esp::gfx::Debug3DText& debug3dText,
                   esp::gfx::DebugRender& debugRender);


  void pour(FluidVesselEntity* other, const Magnum::Vector3& otherSpoutPosWorld, float dt);

  void receiveFluid(const std::string& fluidType, float amount);

 private:
  Blueprint bp_;
  int objId_ = -1;
  esp::sim::Simulator* sim_ = nullptr;
  std::unordered_map<std::string, float> fluidVolumeByType_; // rename to amount
  Corrade::Containers::Optional<Magnum::Vector3> recentPourTarget_;
};

}  // namespace scripted
}  // namespace esp

#endif
