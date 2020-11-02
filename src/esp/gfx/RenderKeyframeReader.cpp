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
                    esp::assets::RenderAssetInstanceCreation>
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

      // todo: load observations!

      keyframes.emplace_back(std::move(keyframe));  // is move needed here?
    }
  } else {
    // todo: warning about no keyframes
  }
}

RenderKeyframeReader::RenderKeyframeReader(
    const LoadAndAddRenderAssetInstanceCallback& callback)
    : loadAndAddRenderAssetInstanceCallback(callback) {}

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
    assetInfos_[assetInfo.filepath] = assetInfo;
  }

  for (const auto& pair : keyframe.creations) {
    const auto& creation = pair.second;
    ASSERT(assetInfos_.count(creation.renderAssetHandle));
    auto node = loadAndAddRenderAssetInstanceCallback(
        assetInfos_[creation.renderAssetHandle], creation);

    const auto& instanceKey = pair.first;
    ASSERT(createdInstances_.count(instanceKey) == 0);
    createdInstances_[instanceKey] = node;
  }

  for (const auto& deletionInstanceKey : keyframe.deletions) {
    auto node = createdInstances_.at(deletionInstanceKey); // todo: use at everywhere?
    delete node;
    createdInstances_.erase(deletionInstanceKey);
  }

  for (const auto& pair : keyframe.stateUpdates) {
    auto node = createdInstances_.at(pair.first); // todo: use at everywhere?
    const auto& state = pair.second;
    node->setTransformation(state.absTransform);
    node->setSemanticId(state.semanticId);
  }
}

}  // namespace gfx
}  // namespace esp
