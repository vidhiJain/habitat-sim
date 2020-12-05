// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "JsonAllTypes.h"

namespace esp {
namespace io {

RJsonValue ToRJsonValue(const esp::gfx::replay::Keyframe& keyframe,
                        RJsonAllocator& allocator) {
  RJsonValue obj(rapidjson::kObjectType);

  esp::io::AddMember(obj, "loads", keyframe.loads, allocator);

  if (!keyframe.creations.empty()) {
    RJsonValue creationsArray(rapidjson::kArrayType);
    for (const auto& pair : keyframe.creations) {
      RJsonValue creationPairObj(rapidjson::kObjectType);
      esp::io::AddMember(creationPairObj, "instanceKey", pair.first, allocator);
      esp::io::AddMember(creationPairObj, "creation", pair.second, allocator);

      creationsArray.PushBack(creationPairObj, allocator);
    }
    esp::io::AddMember(obj, "creations", creationsArray, allocator);
  }

  esp::io::AddMember(obj, "deletions", keyframe.deletions, allocator);

  if (!keyframe.stateUpdates.empty()) {
    RJsonValue stateUpdatesArray(rapidjson::kArrayType);
    for (const auto& pair : keyframe.stateUpdates) {
      RJsonValue stateObj(rapidjson::kObjectType);
      esp::io::AddMember(stateObj, "instanceKey", pair.first, allocator);
      esp::io::AddMember(stateObj, "state", pair.second, allocator);
      stateUpdatesArray.PushBack(stateObj, allocator);
    }
    esp::io::AddMember(obj, "stateUpdates", stateUpdatesArray, allocator);
  }

  if (!keyframe.userTransforms.empty()) {
    RJsonValue userTransformsArray(rapidjson::kArrayType);
    for (const auto& pair : keyframe.userTransforms) {
      RJsonValue wrapperObj(rapidjson::kObjectType);
      esp::io::AddMember(wrapperObj, "name", pair.first, allocator);
      esp::io::AddMember(wrapperObj, "transform", pair.second, allocator);
      userTransformsArray.PushBack(wrapperObj, allocator);
    }
    esp::io::AddMember(obj, "userTransforms", userTransformsArray, allocator);
  }

  return obj;
}

void FromRJsonValue(const RJsonValue& obj,
                    esp::gfx::replay::Keyframe& keyframe) {
  esp::io::ReadMember(obj, "loads", keyframe.loads);

  auto itr = obj.FindMember("creations");
  if (itr != obj.MemberEnd()) {
    const RJsonValue& creationsArray = itr->value;
    keyframe.creations.reserve(creationsArray.Size());
    for (const auto& creationPairObj : creationsArray.GetArray()) {
      std::pair<esp::gfx::replay::RenderAssetInstanceKey,
                esp::assets::RenderAssetInstanceCreationInfo>
          pair;
      esp::io::ReadMember(creationPairObj, "instanceKey", pair.first);
      esp::io::ReadMember(creationPairObj, "creation", pair.second);
      keyframe.creations.emplace_back(std::move(pair));
    }
  }

  esp::io::ReadMember(obj, "deletions", keyframe.deletions);

  itr = obj.FindMember("stateUpdates");
  if (itr != obj.MemberEnd()) {
    const RJsonValue& stateUpdatesArray = itr->value;
    keyframe.stateUpdates.reserve(stateUpdatesArray.Size());
    for (const auto& stateObj : stateUpdatesArray.GetArray()) {
      std::pair<esp::gfx::replay::RenderAssetInstanceKey,
                esp::gfx::replay::RenderAssetInstanceState>
          pair;
      esp::io::ReadMember(stateObj, "instanceKey", pair.first);
      esp::io::ReadMember(stateObj, "state", pair.second);
      keyframe.stateUpdates.emplace_back(std::move(pair));
    }
  }

  itr = obj.FindMember("userTransforms");
  if (itr != obj.MemberEnd()) {
    const RJsonValue& userTransformsArray = itr->value;
    for (const auto& userTransformObj : userTransformsArray.GetArray()) {
      std::string name;
      esp::gfx::replay::Transform transform;
      esp::io::ReadMember(userTransformObj, "name", name);
      esp::io::ReadMember(userTransformObj, "transform", transform);
      keyframe.userTransforms[name] = transform;
    }
  }
}

}  // namespace io
}  // namespace esp
