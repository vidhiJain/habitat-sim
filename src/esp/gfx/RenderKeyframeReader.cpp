// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "RenderKeyframeReader.h"

#include "esp/assets/ResourceManager.h"
#include "esp/io/JsonSerializeTypes.h"
#include "esp/io/json.h"

#include <rapidjson/document.h>
#include <rapidjson/filereadstream.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <iostream>

using namespace rapidjson;

namespace esp {
namespace gfx {

void RenderKeyframeReader::readKeyframesFromJsonDocument(const Document& d) {
  auto& keyframes = keyframes_;

  ASSERT(keyframes.empty());

  Value::ConstMemberIterator itr = d.FindMember("keyframes");
  if (itr != d.MemberEnd()) {
    const Value& keyframesArray = itr->value;
    keyframes.reserve(keyframesArray.Size());
    for (const auto& keyframeObj : keyframesArray.GetArray()) {
      RenderKeyframe keyframe;

      itr = keyframeObj.FindMember("loads");
      if (itr != keyframeObj.MemberEnd()) {
        const Value& loadsArray = itr->value;
        keyframe.loads.reserve(loadsArray.Size());
        for (const auto& loadObj : loadsArray.GetArray()) {
          assets::AssetInfo assetInfo;
          esp::io::ReadMember(loadObj, "assetInfo", assetInfo);
          keyframe.loads.emplace_back(std::move(assetInfo));
        }
      }

      itr = keyframeObj.FindMember("creations");
      if (itr != keyframeObj.MemberEnd()) {
        const Value& creationsArray = itr->value;
        keyframe.creations.reserve(creationsArray.Size());
        for (const auto& creationPairObj : creationsArray.GetArray()) {
          std::pair<RenderAssetInstanceKey,
                    esp::assets::RenderAssetInstanceCreationInfo>
              pair;
          esp::io::ReadMember(creationPairObj, "instanceKey", pair.first);
          esp::io::ReadMember(creationPairObj, "creation", pair.second);
          keyframe.creations.emplace_back(std::move(pair));
        }
      }

      itr = keyframeObj.FindMember("deletions");
      if (itr != keyframeObj.MemberEnd()) {
        const Value& deletionsArray = itr->value;
        keyframe.deletions.reserve(deletionsArray.Size());
        for (const auto& instanceKeyObj : deletionsArray.GetArray()) {
          const auto instanceKey = instanceKeyObj.Get<RenderAssetInstanceKey>();
          keyframe.deletions.push_back(instanceKey);
        }
      }

      itr = keyframeObj.FindMember("stateUpdates");
      if (itr != keyframeObj.MemberEnd()) {
        const Value& stateUpdatesArray = itr->value;
        keyframe.stateUpdates.reserve(stateUpdatesArray.Size());
        for (const auto& stateObj : stateUpdatesArray.GetArray()) {
          std::pair<RenderAssetInstanceKey, RenderAssetInstanceState> pair;
          auto& state = pair.second;

          esp::io::ReadMember(stateObj, "instanceKey", pair.first);
          esp::io::ReadMember(stateObj, "absTransform", state.absTransform);
          esp::io::ReadMember(stateObj, "semanticId", state.semanticId);

          keyframe.stateUpdates.emplace_back(std::move(pair));
        }
      }

      itr = keyframeObj.FindMember("userTransforms");
      if (itr != keyframeObj.MemberEnd()) {
        const Value& userTransformsArray = itr->value;
        for (const auto& userTransformObj : userTransformsArray.GetArray()) {
          std::string name;
          Transform transform;
          esp::io::ReadMember(userTransformObj, "name", name);
          esp::io::ReadMember(userTransformObj, "transform", transform);
          keyframe.userTransforms[name] = transform;
        }
      }

      keyframes.emplace_back(std::move(keyframe));  // is move needed here?
    }
  } else {
    // todo: warning about no keyframes
  }
}

RenderKeyframeReader::RenderKeyframeReader(
    const LoadAndCreateRenderAssetInstanceCallback& callback)
    : loadAndCreateRenderAssetInstanceCallback(callback) {}

void RenderKeyframeReader::readKeyframesFromFile(const std::string& filepath) {
  clearFrame();
  keyframes_.clear();

  auto newDoc = esp::io::parseJsonFile(filepath);
  readKeyframesFromJsonDocument(newDoc);
}

// returns -1 if no frame index set yet
int RenderKeyframeReader::getFrameIndex() {
  return frameIndex_;
}

int RenderKeyframeReader::getNumFrames() {
  return keyframes_.size();
}

void RenderKeyframeReader::setFrame(int frameIndex) {
  ASSERT(frameIndex >= 0 && frameIndex < getNumFrames());

  if (frameIndex < frameIndex_) {
    clearFrame();
  }

  while (frameIndex_ < frameIndex) {
    applyKeyframe(keyframes_[++frameIndex_]);
  }
}

bool RenderKeyframeReader::getUserTransform(const std::string& name,
                                            Magnum::Vector3* translation,
                                            Magnum::Quaternion* rotation) {
  ASSERT(frameIndex_ >= 0 && frameIndex_ < getNumFrames());
  ASSERT(translation);
  ASSERT(rotation);
  const auto& keyframe = keyframes_[frameIndex_];
  const auto& it = keyframe.userTransforms.find(name);
  if (it != keyframe.userTransforms.end()) {
    *translation = it->second.translation;
    *rotation = it->second.rotation;
    return true;
  } else {
    return false;
  }
}

void RenderKeyframeReader::clearFrame() {
  for (const auto& pair : createdInstances_) {
    delete pair.second;
  }
  createdInstances_.clear();
  assetInfos_.clear();
  frameIndex_ = -1;
}

void RenderKeyframeReader::applyKeyframe(const RenderKeyframe& keyframe) {
  for (const auto& assetInfo : keyframe.loads) {
    ASSERT(assetInfos_.count(assetInfo.filepath) == 0);
    if (failedFilepaths_.count(assetInfo.filepath)) {
      continue;
    }
    assetInfos_[assetInfo.filepath] = assetInfo;
  }

  for (const auto& pair : keyframe.creations) {
    const auto& creation = pair.second;
    if (!assetInfos_.count(creation.filepath)) {
      if (!failedFilepaths_.count(creation.filepath)) {
        LOG(WARNING) << "RenderKeyframeReader: missing asset info for ["
                     << creation.filepath << "]";
        failedFilepaths_.insert(creation.filepath);
      }
      continue;
    }
    ASSERT(assetInfos_.count(creation.filepath));
    auto node = loadAndCreateRenderAssetInstanceCallback(
        assetInfos_[creation.filepath], creation);
    if (!node) {
      if (!failedFilepaths_.count(creation.filepath)) {
        LOG(WARNING) << "RenderKeyframeReader: load failed for asset ["
                     << creation.filepath << "]";
        failedFilepaths_.insert(creation.filepath);
      }
      continue;
    }

    const auto& instanceKey = pair.first;
    ASSERT(createdInstances_.count(instanceKey) == 0);
    createdInstances_[instanceKey] = node;
  }

  for (const auto& deletionInstanceKey : keyframe.deletions) {
    const auto& it = createdInstances_.find(deletionInstanceKey);
    if (it == createdInstances_.end()) {
      // missing instance for this key, probably due to a failed instance
      // creation
      continue;
    }

    auto node = it->second;
    delete node;
    createdInstances_.erase(deletionInstanceKey);
  }

  for (const auto& pair : keyframe.stateUpdates) {
    const auto& it = createdInstances_.find(pair.first);
    if (it == createdInstances_.end()) {
      // missing instance for this key, probably due to a failed instance
      // creation
      continue;
    }
    auto node = it->second;
    const auto& state = pair.second;
    node->setTranslation(state.absTransform.translation);
    node->setRotation(state.absTransform.rotation);
    node->setSemanticId(state.semanticId);
  }
}

}  // namespace gfx
}  // namespace esp
