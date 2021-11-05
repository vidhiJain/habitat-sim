// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Config.h"

namespace esp {
namespace arrange {

/*
struct ConfigArticulatedLink {
  float divisionEps = 1e-2f;
  float distEps = 0.02f;
  float maxVelMag = 20.f;
  float baseVelMag0 = 0.2f;
  float baseVelMag1 = 0.05f;
  float rampUpTime = 2.0f;
};
*/

esp::io::JsonGenericValue toJsonValue(const ConfigArticulatedLink& x,
                                      esp::io::JsonAllocator& allocator) {
  esp::io::JsonGenericValue obj(rapidjson::kObjectType);
  esp::io::addMember(obj, "divisionEps", x.divisionEps, allocator);
  esp::io::addMember(obj, "distEps", x.distEps, allocator);
  esp::io::addMember(obj, "maxVelMag", x.maxVelMag, allocator);
  esp::io::addMember(obj, "baseVelMag0", x.baseVelMag0, allocator);
  esp::io::addMember(obj, "baseVelMag1", x.baseVelMag1, allocator);
  esp::io::addMember(obj, "rampUpTime", x.rampUpTime, allocator);
  return obj;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj,
                   ConfigArticulatedLink& x) {
  esp::io::readMember(obj, "divisionEps", x.divisionEps);
  esp::io::readMember(obj, "distEps", x.distEps);
  esp::io::readMember(obj, "maxVelMag", x.maxVelMag);
  esp::io::readMember(obj, "baseVelMag0", x.baseVelMag0);
  esp::io::readMember(obj, "baseVelMag1", x.baseVelMag1);
  esp::io::readMember(obj, "rampUpTime", x.rampUpTime);
  return true;
}

/*
struct ConfigColors {
  Magnum::Color4 goodPlacement = Mn::Color4(0.f, 1.0, 0.f, 1.f);
  Magnum::Color4 badPlacement = Mn::Color4(1.f, 0.0, 0.f, 1.f);
  Magnum::Color4 artObj = Mn::Color4(1.f, 0.5, 0.f, 1.f);
  Magnum::Color4 rigidObj = Mn::Color4(1.f, 0.0, 0.f, 1.f);
};
*/

esp::io::JsonGenericValue toJsonValue(const ConfigColorSet& x,
                                      esp::io::JsonAllocator& allocator) {
  esp::io::JsonGenericValue obj(rapidjson::kObjectType);
  esp::io::addMember(obj, "goodPlacement", x.goodPlacement, allocator);
  esp::io::addMember(obj, "badPlacement", x.badPlacement, allocator);
  esp::io::addMember(obj, "artObj", x.artObj, allocator);
  esp::io::addMember(obj, "rigidObj", x.rigidObj, allocator);
  esp::io::addMember(obj, "placementGuide", x.placementGuide, allocator);
  esp::io::addMember(obj, "debugLines", x.debugLines, allocator);
  return obj;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj, ConfigColorSet& x) {
  esp::io::readMember(obj, "goodPlacement", x.goodPlacement);
  esp::io::readMember(obj, "badPlacement", x.badPlacement);
  esp::io::readMember(obj, "artObj", x.artObj);
  esp::io::readMember(obj, "rigidObj", x.rigidObj);
  esp::io::readMember(obj, "placementGuide", x.placementGuide);
  esp::io::readMember(obj, "debugLines", x.debugLines);
  return true;
}

/*
struct ConfigCamera {
  Magnum::Vector3 eye;
  Magnum::Vector3 lookat;
};*/

esp::io::JsonGenericValue toJsonValue(const ConfigCamera& x,
                                      esp::io::JsonAllocator& allocator) {
  esp::io::JsonGenericValue obj(rapidjson::kObjectType);
  esp::io::addMember(obj, "eye", x.eye, allocator);
  esp::io::addMember(obj, "lookat", x.lookat, allocator);
  return obj;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj, ConfigCamera& x) {
  esp::io::readMember(obj, "eye", x.eye);
  esp::io::readMember(obj, "lookat", x.lookat);
  return true;
}

esp::io::JsonGenericValue toJsonValue(const ConfigLineList& x,
                                      esp::io::JsonAllocator& allocator) {
  esp::io::JsonGenericValue obj(rapidjson::kObjectType);
  esp::io::addMember(obj, "verts", x.verts, allocator);
  return obj;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj, ConfigLineList& x) {
  esp::io::readMember(obj, "verts", x.verts);
  return true;
}

esp::io::JsonGenericValue toJsonValue(const Config& x,
                                      esp::io::JsonAllocator& allocator) {
  esp::io::JsonGenericValue obj(rapidjson::kObjectType);
  esp::io::addMember(obj, "keyframeSavePeriod", x.keyframeSavePeriod,
                     allocator);
  esp::io::addMember(obj, "physicsDeactivationTime", x.physicsDeactivationTime,
                     allocator);
  esp::io::addMember(obj, "physicsMaxSettleTime", x.physicsMaxSettleTime,
                     allocator);
  esp::io::addMember(obj, "linkAnimationMaxDuration",
                     x.linkAnimationMaxDuration, allocator);
  esp::io::addMember(obj, "placementSearchHeight", x.placementSearchHeight,
                     allocator);
  esp::io::addMember(obj, "pickerSphereRadius", x.pickerSphereRadius,
                     allocator);
  esp::io::addMember(obj, "link", x.link, allocator);
  esp::io::addMember(obj, "colors", x.colors, allocator);
  esp::io::addMember(obj, "cameras", x.cameras, allocator);
  return obj;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj, Config& x) {
  esp::io::readMember(obj, "keyframeSavePeriod", x.keyframeSavePeriod);
  esp::io::readMember(obj, "physicsDeactivationTime",
                      x.physicsDeactivationTime);
  esp::io::readMember(obj, "physicsMaxSettleTime", x.physicsMaxSettleTime);
  esp::io::readMember(obj, "linkAnimationMaxDuration",
                      x.linkAnimationMaxDuration);
  esp::io::readMember(obj, "placementSearchHeight", x.placementSearchHeight);
  esp::io::readMember(obj, "link", x.link);
  esp::io::readMember(obj, "colors", x.colors);
  esp::io::readMember(obj, "cameras", x.cameras);
  esp::io::readMember(obj, "debugLineLists", x.debugLineLists);
  return true;
}

}  // namespace arrange
}  // namespace esp
