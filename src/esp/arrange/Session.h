// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_ARRANGE_SESSION_H_
#define ESP_ARRANGE_SESSION_H_

#include <string>
#include <vector>

#include "esp/gfx/replay/Keyframe.h"  // for Transform
#include "esp/io/JsonAllTypes.h"
#include "esp/physics/PhysicsKeyframe.h"

#include "Config.h"

namespace esp {
namespace arrange {

struct UserAction {
  bool isSettlingAction = false;
  std::string rigidObj;
  std::string articulatedObj;
  int articulatedLink = -1;
  int startFrame = -1;
  int endFrame = -1;  // action *includes* this frame
};

struct Session {
  std::string dataset;
  std::string scene;
  float physicsTimeStep = 0.f;
  // an optional static camera that provides a reasonabe view of the session
  Corrade::Containers::Optional<esp::gfx::replay::Transform> defaultCamera;
  Config config;
  std::vector<UserAction> userActions;
  std::vector<esp::physics::PhysicsKeyframe> keyframes;
};

esp::io::JsonGenericValue toJsonValue(const UserAction& x,
                                      esp::io::JsonAllocator& allocator);

bool fromJsonValue(const esp::io::JsonGenericValue& obj, UserAction& x);

esp::io::JsonGenericValue toJsonValue(const Session& x,
                                      esp::io::JsonAllocator& allocator);

bool fromJsonValue(const esp::io::JsonGenericValue& obj, Session& x);

}  // namespace arrange
}  // namespace esp

#endif