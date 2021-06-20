// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/io/JsonAllTypes.h"

#include "Session.h"

namespace esp {
namespace arrange {

/*
struct UserAction {
  bool isInitialSettling = false;
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
  esp::io::addMember(obj, "isInitialSettling", x.isInitialSettling, allocator);
  esp::io::addMember(obj, "rigidObj", x.rigidObj, allocator);
  esp::io::addMember(obj, "articulatedObj", x.articulatedObj, allocator);
  esp::io::addMember(obj, "articulatedLink", x.articulatedLink, allocator);
  esp::io::addMember(obj, "startFrame", x.startFrame, allocator);
  esp::io::addMember(obj, "endFrame", x.endFrame, allocator);
  return obj;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj, UserAction& x) {
  esp::io::readMember(obj, "isInitialSettling", x.isInitialSettling);
  esp::io::readMember(obj, "rigidObj", x.rigidObj);
  esp::io::readMember(obj, "articulatedObj", x.articulatedObj);
  esp::io::readMember(obj, "articulatedLink", x.articulatedLink);
  esp::io::readMember(obj, "startFrame", x.startFrame);
  esp::io::readMember(obj, "endFrame", x.endFrame);
  return true;
}

/*
struct Session {
  std::string scene;
  std::vector<UserAction> userActions;
  std::vector<esp::physics::PhysicsKeyframe> keyframes;
};
*/

esp::io::JsonGenericValue toJsonValue(const Session& x,
                                      esp::io::JsonAllocator& allocator) {
  esp::io::JsonGenericValue obj(rapidjson::kObjectType);
  esp::io::addMember(obj, "scene", x.scene, allocator);
  esp::io::addMember(obj, "userActions", x.userActions, allocator);
  esp::io::addMember(obj, "keyframes", x.keyframes, allocator);
  return obj;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj, Session& x) {}

}  // namespace arrange
}  // namespace esp
