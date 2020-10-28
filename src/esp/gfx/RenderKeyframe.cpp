// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "RenderKeyframe.h"

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

// hang on to newNode
// set up persistent name/id for newNode
// record transforms over time
// todo: need to know if RGB or semantic graph
void RenderKeyframeWriter::onCreateRenderAssetInstance(
    scene::SceneNode* node,
    const esp::assets::AssetInfo& assetInfo,
    bool isSemantic,
    bool isRGBD,
    Magnum::ResourceKey lightSetupKey) {
  ASSERT(node);

#if 0
  NodeKey key = node->getUniqueId();

  ASSERT(instanceRecords_.count(key) == 0);
  instanceRecords_[key] = RenderAssetInstanceRecord();
#endif

  ASSERT(findInstance(node) == -1);

  RenderAssetInstanceKey instanceKey = getNewInstanceKey();

  getKeyframe().creations.emplace_back(
      RenderAssetInstanceCreation{.instanceKey = instanceKey,
                                  .assetInfo = assetInfo,
                                  .isSemantic = isSemantic,
                                  .isRGBD = isRGBD,
                                  .lightSetupKey = lightSetupKey});

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

  // advance to next keyframe
  keyframes_.emplace_back();
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
#if 0
  using MagnumObjectContainer = std::vector<std::reference_wrapper<MagnumObject>>;
  std::unordered_map<const MagnumScene*, MagnumObjectContainer> objectsByScene;

  for (const auto& instanceRecord : instanceRecords_) {

    const auto* scene = dynamic_cast<const MagnumScene*>(instanceRecord.node->scene());

    objectsByScene[scene].push_back(*instanceRecord.node);
  }

  for (const auto& pair : objectsByScene) {

    const auto* scene = pair.first;
    const auto& objects = pair.second;

    std::vector<Magnum::Matrix4> transformations =
        scene->transformationMatrices(objects);

  }
#endif

  for (auto& instanceRecord : instanceRecords_) {
    auto state = getInstanceState(instanceRecord.node);
    if (!instanceRecord.recentState || state != instanceRecord.recentState) {
      getKeyframe().stateUpdates.push_back(
          std::make_pair(instanceRecord.instanceKey, state));
      instanceRecord.recentState = state;
    }
  }
}

}  // namespace gfx
}  // namespace esp
