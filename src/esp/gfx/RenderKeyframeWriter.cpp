// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "RenderKeyframeWriter.h"

#include "esp/io/JsonSerializeTypes.h"
#include "esp/io/json.h"

#include <rapidjson/document.h>
#include <rapidjson/filewritestream.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <iostream>

using namespace rapidjson;

namespace esp {
namespace gfx {

// todo: find less bloated way to get deletion notification
class NodeDeletionHelper : public Magnum::SceneGraph::AbstractFeature3D {
 public:
  NodeDeletionHelper(scene::SceneNode& node_, RenderKeyframeWriter* writer)
      : Magnum::SceneGraph::AbstractFeature3D(node_),
        node(&node_),
        writer_(writer) {}

  virtual ~NodeDeletionHelper() { writer_->onDeleteRenderAssetInstance(node); }

 private:
  RenderKeyframeWriter* writer_ = nullptr;
  const scene::SceneNode* node = nullptr;
};

void RenderKeyframeWriter::onLoadRenderAsset(
    const esp::assets::AssetInfo& assetInfo) {
  // todo: store this elsewhere, not per keyframe, and only write those that get used 
  // for render asset instances. For example, we want to avoid writing render assets
  // used exclusively for collision.
  getKeyframe().loads.push_back(assetInfo);
}

// hang on to newNode
// set up persistent name/id for newNode
// record transforms over time
// todo: need to know if RGB or semantic graph
void RenderKeyframeWriter::onCreateRenderAssetInstance(
    scene::SceneNode* node,
    const esp::assets::RenderAssetInstanceCreation& creation) {
  ASSERT(node);
  ASSERT(findInstance(node) == -1);

  RenderAssetInstanceKey instanceKey = getNewInstanceKey();

  getKeyframe().creations.emplace_back(std::make_pair(instanceKey, creation));

  instanceRecords_.emplace_back(
      RenderAssetInstanceRecord{.node = node, .instanceKey = instanceKey});

  node->addFeature<NodeDeletionHelper>(this);
}

void RenderKeyframeWriter::onDrawObservation(
    const sensor::VisualSensor& visualSensor) {
  updateInstanceStates();

  getKeyframe().observation = ObservationRecord{
      .sensorType = visualSensor.specification()->sensorType,
      .cameraTransform = visualSensor.node().absoluteTransformationMatrix()};

  advanceKeyframe();
}

void RenderKeyframeWriter::onDeleteRenderAssetInstance(
    const scene::SceneNode* node) {
  int index = findInstance(node);
  ASSERT(index != -1);

  auto instanceKey = instanceRecords_[index].instanceKey;

  getKeyframe().deletions.push_back(instanceKey);

  instanceRecords_.erase(instanceRecords_.begin() + index);
}

RenderKeyframe& RenderKeyframeWriter::getKeyframe() {
  // perf todo: avoid growing keyframes vector this way; it will be slow
  if (keyframes_.empty()) {
    keyframes_.emplace_back();
  }

  return keyframes_.back();
}

RenderAssetInstanceKey RenderKeyframeWriter::getNewInstanceKey() {
  return nextInstanceKey_++;
}

int RenderKeyframeWriter::findInstance(const scene::SceneNode* queryNode) {
  auto it = std::find_if(instanceRecords_.begin(), instanceRecords_.end(),
                         [&queryNode](const RenderAssetInstanceRecord& record) {
                           return record.node == queryNode;
                         });

  return it == instanceRecords_.end() ? -1 : int(it - instanceRecords_.begin());
}

RenderAssetInstanceState RenderKeyframeWriter::getInstanceState(
    const scene::SceneNode* node) {
  return RenderAssetInstanceState{
      // .absTranslation = absTransform.translation(),
      // .absRotation = absTransform.rotation(),
      // .absScaling = absTransform.scaling(),
      .absTransform = node->absoluteTransformation(),
      .semanticId = node->getSemanticId()};
}

void RenderKeyframeWriter::updateInstanceStates() {
  for (auto& instanceRecord : instanceRecords_) {
    auto state = getInstanceState(instanceRecord.node);
    if (!instanceRecord.recentState || state != instanceRecord.recentState) {
      getKeyframe().stateUpdates.push_back(
          std::make_pair(instanceRecord.instanceKey, state));
      instanceRecord.recentState = state;
    }
  }
}

void RenderKeyframeWriter::advanceKeyframe() {
  keyframes_.emplace_back();

  // temp
  if (instanceRecords_.size() == 4) {
    writeKeyframesToFile("my_replay.json");
    _exit(0);
  }
}

void RenderKeyframeWriter::writeKeyframesToFile(const std::string& filepath) {
  auto document = writeKeyframesToJsonDocument();
  esp::io::writeJsonToFile(document, filepath);
}
/*
struct RenderAssetInstanceCreation {
  RenderAssetInstanceKey instanceKey;
  esp::assets::AssetInfo assetInfo;
  bool isSemantic;
  bool isRGBD;
  Magnum::ResourceKey lightSetupKey;
};

struct RenderAssetInstanceState {
  Magnum::Matrix4 absTransform;  // todo: Matrix4 or Matrix4x4?
  // todo: support semanticId per drawable?
  int semanticId = -1;
  // todo: support mutable lightSetupKey?

  bool operator==(const RenderAssetInstanceState& rhs) const {
    return absTransform == rhs.absTransform && semanticId == rhs.semanticId;
  }
};
*/

rapidjson::Document RenderKeyframeWriter::writeKeyframesToJsonDocument() {
  if (keyframes_.empty()) {
    // todo: warn about nothing to write
    return rapidjson::Document();
  }

  Document d;
  d.SetObject();
  Document::AllocatorType& allocator = d.GetAllocator();

  Value keyframesArray(kArrayType);
  for (const auto& keyframe : keyframes_) {
    Value keyframeObj(kObjectType);

    if (!keyframe.loads.empty()) {
      Value loadsArray(kArrayType);
      for (const auto& assetInfo : keyframe.loads) {
        Value loadObj(kObjectType);
        esp::io::AddMember(loadObj, "assetInfo", assetInfo, allocator);
        loadsArray.PushBack(loadObj, allocator);
      }
      esp::io::AddMember(keyframeObj, "loads", loadsArray, allocator);
    }

    if (!keyframe.creations.empty()) {
      Value creationsArray(kArrayType);
      for (const auto& pair : keyframe.creations) {
        Value creationPairObj(kObjectType);
        esp::io::AddMember(creationPairObj, "instanceKey", pair.first,
                           allocator);
        esp::io::AddMember(creationPairObj, "creation", pair.second, allocator);

        creationsArray.PushBack(creationPairObj, allocator);
      }
      esp::io::AddMember(keyframeObj, "creations", creationsArray, allocator);
    }

    if (!keyframe.deletions.empty()) {
      Value deletionsArray(kArrayType);
      for (const auto& deletionInstanceKey : keyframe.deletions) {
        deletionsArray.PushBack(deletionInstanceKey, allocator);
      }
      esp::io::AddMember(keyframeObj, "deletions", deletionsArray, allocator);
    }

    if (!keyframe.stateUpdates.empty()) {
      Value stateUpdatesArray(kArrayType);
      for (const auto& pair : keyframe.stateUpdates) {
        const auto& state = pair.second;

        Value stateObj(kObjectType);
        esp::io::AddMember(stateObj, "instanceKey", pair.first, allocator);
        esp::io::AddMember(stateObj, "absTransform", state.absTransform,
                           allocator);
        esp::io::AddMember(stateObj, "semanticId", state.semanticId, allocator);

        stateUpdatesArray.PushBack(stateObj, allocator);
      }
      esp::io::AddMember(keyframeObj, "stateUpdates", stateUpdatesArray,
                         allocator);
    }

    if (keyframe.observation) {
      const auto& obs = *keyframe.observation;
      Value obsObj(kObjectType);
      esp::io::AddMember(obsObj, "cameraTransform", obs.cameraTransform,
                         allocator);
      esp::io::AddMemberEnum(obsObj, "sensorType", obs.sensorType, allocator);
      esp::io::AddMember(keyframeObj, "observation", obsObj, allocator);
    }

    keyframesArray.PushBack(keyframeObj, allocator);
  }

  d.AddMember("keyframes", keyframesArray, allocator);
  return d;
}

}  // namespace gfx
}  // namespace esp
