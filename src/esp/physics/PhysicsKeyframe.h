// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_PHYSICSKEYFRAME_H_
#define ESP_PHYSICS_PHYSICSKEYFRAME_H_

#include <string>
#include <vector>

#include <Magnum/Magnum.h>
#include <Magnum/Math/Quaternion.h>
#include <Magnum/Math/Vector3.h>

namespace esp {
namespace physics {

struct RigidObjectKeyframe {
  std::string name;
  Magnum::Vector3 translation;
  Magnum::Quaternion rotation;
};

struct ArticulatedObjectKeyframe {
  std::string name;
  Magnum::Vector3 translation;
  Magnum::Quaternion rotation;
  std::vector<float> jointPositions;
};

struct PhysicsKeyframe {
  std::vector<RigidObjectKeyframe> rigidObjects;
  std::vector<ArticulatedObjectKeyframe> articulatedObjects;
};

}  // namespace physics
}  // namespace esp

#endif
