// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Config.h"

namespace esp {
namespace arrange {

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

/*
struct Config {
  std::vector<ConfigCamera> cameras;
};*/

esp::io::JsonGenericValue toJsonValue(const Config& x,
                                      esp::io::JsonAllocator& allocator) {
  esp::io::JsonGenericValue obj(rapidjson::kObjectType);
  esp::io::addMember(obj, "cameras", x.cameras, allocator);
  return obj;
}

bool fromJsonValue(const esp::io::JsonGenericValue& obj, Config& x) {
  esp::io::readMember(obj, "cameras", x.cameras);
  return true;
}

}  // namespace arrange
}  // namespace esp
