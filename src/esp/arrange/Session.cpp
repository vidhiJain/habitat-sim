// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/io/JsonAllTypes.h"

#include "Session.h"

namespace esp {
namespace arrange {

/*
struct UserAction {
  bool isSettlingAction = false;
  std::string rigidObj;
  std::string articulatedObj;
  int articulatedLink = -1;
  int startFrame = -1;
  int endFrame = -1;
};
*/

esp::io::JsonGenericValue toJsonValue(const UserAction& x,
                                      esp::io::JsonAllocator& allocator) {
  esp::io::JsonGenericValue obj(rapidjson::kObjectType);
  esp::io::addMember(obj, "isSettlingAction", x.isSettlingAction, allocator);
  esp::io::addMember(obj, "rigidObj", x.rigidObj, allocator);
  esp::io::addMember(obj, "articulatedObj", x.articulatedObj, allocator);
  esp::io::addMember(obj, "articulatedLink", x.articulatedLink, allocator);
  esp::io::addMember(obj, "startFrame", x.startFrame, allocator);
  esp::io::addMember(obj, "endFrame", x.endFrame, allocator);
  return obj;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj, UserAction& x) {
  esp::io::readMember(obj, "isSettlingAction", x.isSettlingAction);
  esp::io::readMember(obj, "rigidObj", x.rigidObj);
  esp::io::readMember(obj, "articulatedObj", x.articulatedObj);
  esp::io::readMember(obj, "articulatedLink", x.articulatedLink);
  esp::io::readMember(obj, "startFrame", x.startFrame);
  esp::io::readMember(obj, "endFrame", x.endFrame);
  return true;
}

/*
struct Session {
  std::string dataset;
  std::string scene;
  Config config;
  float physicsTimeStep = 0.f;
  // an optional static camera that provides a reasonabe view of the session
  Corrade::Containers::Optional<ConfigCamera> defaultCamera;
  std::vector<UserAction> userActions;
  std::vector<esp::physics::PhysicsKeyframe> keyframes;
};
*/

esp::io::JsonGenericValue toJsonValue(const Session& x,
                                      esp::io::JsonAllocator& allocator) {
  esp::io::JsonGenericValue obj(rapidjson::kObjectType);
  esp::io::addMember(obj, "dataset", x.dataset, allocator);
  esp::io::addMember(obj, "scene", x.scene, allocator);
  esp::io::addMember(obj, "physicsTimeStep", x.physicsTimeStep, allocator);
  esp::io::addMember(obj, "defaultCamera", x.defaultCamera, allocator);
  esp::io::addMember(obj, "config", x.config, allocator);
  esp::io::addMember(obj, "userActions", x.userActions, allocator);
  esp::io::addMember(obj, "keyframes", x.keyframes, allocator);
  return obj;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj, Session& x) {
  esp::io::readMember(obj, "dataset", x.dataset);
  esp::io::readMember(obj, "scene", x.scene);
  esp::io::readMember(obj, "physicsTimeStep", x.physicsTimeStep);
  esp::io::readMember(obj, "defaultCamera", x.defaultCamera);
  esp::io::readMember(obj, "config", x.config);
  esp::io::readMember(obj, "userActions", x.userActions);
  esp::io::readMember(obj, "keyframes", x.keyframes);
  return true;
}

}  // namespace arrange
}  // namespace esp
